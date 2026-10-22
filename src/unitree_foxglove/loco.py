"""Stand-in for the G1's onboard locomotion service, for the simulator.

Runs Unitree's pretrained G1 walking policy from ``unitree_rl_gym`` against the
same ``rt/lowstate`` / ``rt/lowcmd`` topics the MuJoCo bridge speaks. The
observation layout, gains, and scales are the ones in
``deploy/deploy_real/configs/g1.yaml`` and ``deploy_real.py``; that script is not
imported because it depends on ``legged_gym`` and Isaac Gym.

Modes, requested on the ``loco/command`` DDS topic:

- ``damp``: zero stiffness, light damping on every motor (safe stop, initial)
- ``stand``: two-second ramp from the current joints to the default pose, then hold
- ``walk``: the policy at 50 Hz with the commanded body velocity

On a real G1 the same requests map to ``LocoClient`` (``Damp``, ``StandUp``,
``Start``, ``Move``); see ``bridge.py``.
"""

from __future__ import annotations

import os
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.utils.crc import CRC

from unitree_foxglove.idl import (
    MODE_DAMP,
    MODE_STAND,
    MODE_WALK,
    MODES,
    TOPIC_LOCO_COMMAND,
    LocoCommand,
)
from unitree_foxglove.robots import G1, RobotProfile

COMMAND_TIMEOUT_S = 0.5
STAND_RAMP_S = 2.0
DAMPING_KD = 8.0
GAIT_PERIOD_S = 0.8


@dataclass(frozen=True)
class PolicyConfig:
    """Values from ``unitree_rl_gym/deploy/deploy_real/configs/g1.yaml``."""

    control_dt: float
    policy_path: Path
    leg_joint2motor_idx: list[int]
    kps: list[float]
    kds: list[float]
    default_angles: np.ndarray
    arm_waist_joint2motor_idx: list[int]
    arm_waist_kps: list[float]
    arm_waist_kds: list[float]
    arm_waist_target: np.ndarray
    ang_vel_scale: float
    dof_pos_scale: float
    dof_vel_scale: float
    action_scale: float
    cmd_scale: np.ndarray
    max_cmd: np.ndarray
    num_actions: int
    num_obs: int

    @classmethod
    def load(cls, config_path: Path, policy_path: Path) -> PolicyConfig:
        raw = yaml.safe_load(config_path.read_text())
        return cls(
            control_dt=float(raw["control_dt"]),
            policy_path=policy_path,
            leg_joint2motor_idx=list(raw["leg_joint2motor_idx"]),
            kps=[float(v) for v in raw["kps"]],
            kds=[float(v) for v in raw["kds"]],
            default_angles=np.array(raw["default_angles"], dtype=np.float32),
            arm_waist_joint2motor_idx=list(raw["arm_waist_joint2motor_idx"]),
            arm_waist_kps=[float(v) for v in raw["arm_waist_kps"]],
            arm_waist_kds=[float(v) for v in raw["arm_waist_kds"]],
            arm_waist_target=np.array(raw["arm_waist_target"], dtype=np.float32),
            ang_vel_scale=float(raw["ang_vel_scale"]),
            dof_pos_scale=float(raw["dof_pos_scale"]),
            dof_vel_scale=float(raw["dof_vel_scale"]),
            action_scale=float(raw["action_scale"]),
            cmd_scale=np.array(raw["cmd_scale"], dtype=np.float32),
            max_cmd=np.array(raw["max_cmd"], dtype=np.float32),
            num_actions=int(raw["num_actions"]),
            num_obs=int(raw["num_obs"]),
        )


def gravity_orientation(quaternion_wxyz: np.ndarray) -> np.ndarray:
    """Gravity direction in the body frame, as ``rotation_helper.py`` computes it."""
    qw, qx, qy, qz = (float(v) for v in quaternion_wxyz[:4])
    return np.array(
        [
            2 * (-qz * qx + qw * qy),
            -2 * (qz * qy + qw * qx),
            1 - 2 * (qw * qw + qz * qz),
        ],
        dtype=np.float32,
    )


def clamp_command(config: PolicyConfig, vx: float, vy: float, wz: float) -> np.ndarray:
    """Body velocity limited to what the policy was trained for."""
    raw = np.array([vx, vy, wz], dtype=np.float32)
    return np.clip(raw, -config.max_cmd, config.max_cmd)


def build_observation(
    config: PolicyConfig,
    *,
    joint_pos: np.ndarray,
    joint_vel: np.ndarray,
    quaternion_wxyz: np.ndarray,
    gyroscope: np.ndarray,
    command: np.ndarray,
    last_action: np.ndarray,
    step: int,
) -> np.ndarray:
    """Assemble the 47-entry observation vector in the upstream order."""
    n = config.num_actions
    obs = np.zeros(config.num_obs, dtype=np.float32)
    phase = (step * config.control_dt) % GAIT_PERIOD_S / GAIT_PERIOD_S
    obs[0:3] = np.asarray(gyroscope, dtype=np.float32) * config.ang_vel_scale
    obs[3:6] = gravity_orientation(quaternion_wxyz)
    obs[6:9] = command * config.cmd_scale
    obs[9 : 9 + n] = (joint_pos - config.default_angles) * config.dof_pos_scale
    obs[9 + n : 9 + 2 * n] = joint_vel * config.dof_vel_scale
    obs[9 + 2 * n : 9 + 3 * n] = last_action
    obs[9 + 3 * n] = np.sin(2 * np.pi * phase)
    obs[9 + 3 * n + 1] = np.cos(2 * np.pi * phase)
    return obs


class Policy:
    """TorchScript policy loaded on the CPU."""

    def __init__(self, path: Path) -> None:
        import torch

        torch.set_num_threads(1)
        self._torch = torch
        self._module = torch.jit.load(str(path), map_location="cpu")
        self._module.eval()

    def act(self, observation: np.ndarray) -> np.ndarray:
        tensor = self._torch.from_numpy(observation).unsqueeze(0)
        with self._torch.no_grad():
            action = self._module(tensor)
        return action.numpy().squeeze().astype(np.float32)


@dataclass
class CommandState:
    """Latest request from the bridge, with the wall time it arrived."""

    mode: str = MODE_DAMP
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    received_at: float = 0.0

    def current_velocity(self, now: float) -> np.ndarray:
        if now - self.received_at > COMMAND_TIMEOUT_S:
            return np.zeros(3, dtype=np.float32)
        return self.velocity


class LocoController:
    """DDS wiring plus the damp / stand / walk state machine."""

    def __init__(self, config: PolicyConfig, profile: RobotProfile, policy: Policy) -> None:
        self.config = config
        self.profile = profile
        self.policy = policy
        self.crc = CRC()
        self.low_cmd = profile.new_low_cmd()
        self.low_state: object | None = None
        self.state_lock = threading.Lock()
        self.command = CommandState()
        self.command_lock = threading.Lock()
        self.active_mode = MODE_DAMP
        self.stand_ramp_done = False
        self.ramp_start: np.ndarray | None = None
        self.ramp_started_at = 0.0
        self.last_action = np.zeros(config.num_actions, dtype=np.float32)
        self.step = 0

        self.publisher = ChannelPublisher("rt/lowcmd", profile.low_cmd_type())
        self.publisher.Init()
        self.state_sub = ChannelSubscriber("rt/lowstate", profile.low_state_type())
        self.state_sub.Init(self._on_low_state, 10)
        self.command_sub = ChannelSubscriber(TOPIC_LOCO_COMMAND, LocoCommand)
        self.command_sub.Init(self._on_command, 10)

    def _on_low_state(self, msg: object) -> None:
        with self.state_lock:
            self.low_state = msg

    def _on_command(self, msg: LocoCommand) -> None:
        if msg.mode not in MODES:
            print(f"ignoring unknown loco mode {msg.mode!r}", flush=True)
            return
        with self.command_lock:
            self.command.mode = msg.mode
            self.command.velocity = clamp_command(self.config, msg.vx, msg.vy, msg.wz)
            self.command.received_at = time.monotonic()

    def wait_for_state(self, stop: threading.Event) -> bool:
        while not stop.is_set():
            with self.state_lock:
                if self.low_state is not None:
                    return True
            time.sleep(self.config.control_dt)
        return False

    def tick(self) -> None:
        with self.state_lock:
            low_state = self.low_state
        with self.command_lock:
            requested = self.command.mode
            velocity = self.command.current_velocity(time.monotonic())
        if low_state is None:
            return
        if requested != self.active_mode:
            self._enter(requested, low_state)
        if self.active_mode == MODE_DAMP:
            self._damp()
        elif self.active_mode == MODE_STAND or not self.stand_ramp_done:
            self._stand(low_state)
        else:
            self._walk(low_state, velocity)
        self._send(low_state)

    def _enter(self, mode: str, low_state: object) -> None:
        print(f"loco mode {self.active_mode} -> {mode}", flush=True)
        if mode in (MODE_STAND, MODE_WALK) and self.active_mode == MODE_DAMP:
            self._start_ramp(low_state)
        if mode == MODE_DAMP:
            self.stand_ramp_done = False
        if mode == MODE_WALK:
            self.last_action[:] = 0.0
            self.step = 0
        self.active_mode = mode

    def _start_ramp(self, low_state: object) -> None:
        indices = self.config.leg_joint2motor_idx + self.config.arm_waist_joint2motor_idx
        motors = low_state.motor_state  # type: ignore[attr-defined]
        self.ramp_start = np.array([motors[i].q for i in indices], dtype=np.float32)
        self.ramp_started_at = time.monotonic()
        self.stand_ramp_done = False

    def _damp(self) -> None:
        for motor in self.low_cmd.motor_cmd:  # type: ignore[attr-defined]
            motor.q = 0.0
            motor.dq = 0.0
            motor.kp = 0.0
            motor.kd = DAMPING_KD
            motor.tau = 0.0

    def _stand(self, low_state: object) -> None:
        if self.ramp_start is None:
            self._start_ramp(low_state)
        assert self.ramp_start is not None
        target = np.concatenate((self.config.default_angles, self.config.arm_waist_target))
        alpha = min(1.0, (time.monotonic() - self.ramp_started_at) / STAND_RAMP_S)
        if alpha >= 1.0 and not self.stand_ramp_done:
            self.stand_ramp_done = True
            print("default pose reached", flush=True)
        pose = self.ramp_start * (1 - alpha) + target * alpha
        indices = self.config.leg_joint2motor_idx + self.config.arm_waist_joint2motor_idx
        kps = self.config.kps + self.config.arm_waist_kps
        kds = self.config.kds + self.config.arm_waist_kds
        self._set_targets(indices, pose, kps, kds)

    def _walk(self, low_state: object, velocity: np.ndarray) -> None:
        motors = low_state.motor_state  # type: ignore[attr-defined]
        imu = low_state.imu_state  # type: ignore[attr-defined]
        legs = self.config.leg_joint2motor_idx
        joint_pos = np.array([motors[i].q for i in legs], dtype=np.float32)
        joint_vel = np.array([motors[i].dq for i in legs], dtype=np.float32)
        self.step += 1
        observation = build_observation(
            self.config,
            joint_pos=joint_pos,
            joint_vel=joint_vel,
            quaternion_wxyz=np.array(imu.quaternion, dtype=np.float32),
            gyroscope=np.array(imu.gyroscope, dtype=np.float32),
            command=velocity,
            last_action=self.last_action,
            step=self.step,
        )
        self.last_action = self.policy.act(observation)
        leg_targets = self.config.default_angles + self.last_action * self.config.action_scale
        self._set_targets(legs, leg_targets, self.config.kps, self.config.kds)
        self._set_targets(
            self.config.arm_waist_joint2motor_idx,
            self.config.arm_waist_target,
            self.config.arm_waist_kps,
            self.config.arm_waist_kds,
        )

    def _set_targets(
        self, indices: list[int], targets: np.ndarray, kps: list[float], kds: list[float]
    ) -> None:
        for slot, motor_index in enumerate(indices):
            motor = self.low_cmd.motor_cmd[motor_index]  # type: ignore[attr-defined]
            motor.q = float(targets[slot])
            motor.dq = 0.0
            motor.kp = float(kps[slot])
            motor.kd = float(kds[slot])
            motor.tau = 0.0

    def _send(self, low_state: object) -> None:
        if self.profile.idl == "hg":
            self.low_cmd.mode_pr = 0  # type: ignore[attr-defined]
            self.low_cmd.mode_machine = low_state.mode_machine  # type: ignore[attr-defined]
            for motor in self.low_cmd.motor_cmd:  # type: ignore[attr-defined]
                motor.mode = 1
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)  # type: ignore[attr-defined]
        self.publisher.Write(self.low_cmd)


def default_paths() -> tuple[Path, Path]:
    """Config and policy from the pinned ``unitree_rl_gym`` checkout."""
    root = Path(
        os.environ.get(
            "UNITREE_RL_GYM_ROOT",
            Path(__file__).resolve().parents[2] / "third_party" / "unitree_rl_gym",
        )
    )
    return (
        root / "deploy" / "deploy_real" / "configs" / "g1.yaml",
        root / "deploy" / "pre_train" / "g1" / "motion.pt",
    )


def main() -> None:
    """Run the controller until SIGTERM."""
    domain_id = int(os.environ.get("UNITREE_DOMAIN_ID", "1"))
    interface = os.environ.get("UNITREE_INTERFACE", "lo")
    robot = os.environ.get("UNITREE_ROBOT", G1.name)
    if robot != G1.name:
        print(f"The walking policy supports only g1, not {robot!r}. (LOCO-001)", file=sys.stderr)
        raise SystemExit(64)
    config_path, policy_path = default_paths()
    for path in (config_path, policy_path):
        if not path.is_file():
            print(f"Missing {path}. Initialise the unitree_rl_gym submodule. (LOCO-002)")
            raise SystemExit(66)

    stop = threading.Event()

    def handler(signum: int, _frame: object) -> None:
        print(f"stopping loco on signal {signum}", flush=True)
        stop.set()

    signal.signal(signal.SIGTERM, handler)
    signal.signal(signal.SIGINT, handler)

    config = PolicyConfig.load(config_path, policy_path)
    policy = Policy(policy_path)
    ChannelFactoryInitialize(domain_id, interface)
    controller = LocoController(config, G1, policy)
    print(f"loco waiting for rt/lowstate on domain {domain_id} interface {interface}", flush=True)
    if not controller.wait_for_state(stop):
        os._exit(0)
    print("loco connected, mode damp; send loco/command to stand or walk", flush=True)

    # Absolute schedule keeps the policy at exactly control_dt on average.
    next_tick = time.perf_counter()
    while not stop.is_set():
        controller.tick()
        next_tick += config.control_dt
        remaining = next_tick - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)
        elif remaining < -0.1:
            next_tick = time.perf_counter()  # fell far behind; do not burst
    controller.command.mode = MODE_DAMP
    with controller.state_lock:
        low_state = controller.low_state
    if low_state is not None:
        controller._damp()
        controller._send(low_state)
    os._exit(0)
