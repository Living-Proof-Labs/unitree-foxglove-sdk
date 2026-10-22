"""Run the Unitree MuJoCo simulator and publish its state over DDS.

Configuration comes from the environment so the upstream ``config.py`` stays
untouched. Joystick emulation stays off: the upstream bridge exits when no
gamepad is attached.

Humanoids start hanging from an elastic band, as upstream does, so a controller
can take over before the robot falls. The band answers viewer keys 7 / 8 / 9
(shorten, lengthen, toggle) and the ``sim/command`` DDS topic, which the Foxglove
bridge publishes on behalf of the Service Call panel.

``UNITREE_WORLD`` selects walls to add around the upstream scene (see
``worlds.py``). A lidar fan on the torso publishes ``sensor/scan`` at 10 Hz in
every world, so the navigation stack sees the same message in the open and in a
hallway.
"""

from __future__ import annotations

import os
import signal
import sys
import threading
import time
import types
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from unitree_foxglove.idl import TOPIC_SCAN, TOPIC_SIM_COMMAND, LaserScan, SimCommand
from unitree_foxglove.robots import DEFAULT_ROBOT, RobotProfile, profile_for
from unitree_foxglove.worlds import (
    DEFAULT_WORLD,
    LIDAR,
    World,
    build_model,
    lidar_sensor_slice,
    ranges_from_sensordata,
    world_for,
)

# The physics step stays at the model's own value (MuJoCo default 2 ms).
PUBLISH_DT = 0.005
SCAN_DT = 0.1
VIEWER_DT = 0.02
BAND_STEP_M = 0.1
RTF_REPORT_S = 5.0
MAX_CATCH_UP_S = 0.05

_KEY_7, _KEY_8, _KEY_9 = 55, 56, 57


@dataclass(frozen=True)
class Settings:
    """Simulator settings read from the environment."""

    robot: RobotProfile
    domain_id: int
    interface: str
    headless: bool
    duration_s: float
    scene: Path
    mujoco_root: Path
    elastic_band: bool
    world: World | None = None

    @classmethod
    def from_env(cls) -> Settings:
        """Read settings, resolving the robot scene under UNITREE_MUJOCO_ROOT."""
        robot = profile_for(os.environ.get("UNITREE_ROBOT", DEFAULT_ROBOT))
        root = Path(os.environ.get("UNITREE_MUJOCO_ROOT", _default_mujoco_root()))
        scene = root / "unitree_robots" / robot.name / robot.scene_file
        band_env = os.environ.get("UNITREE_ELASTIC_BAND")
        world = world_for(os.environ.get("UNITREE_WORLD", DEFAULT_WORLD))
        return cls(
            robot=robot,
            domain_id=int(os.environ.get("UNITREE_DOMAIN_ID", "1")),
            interface=os.environ.get("UNITREE_INTERFACE", "lo"),
            headless=_as_bool(os.environ.get("UNITREE_HEADLESS", "0")),
            duration_s=float(os.environ.get("UNITREE_DURATION", "0")),
            scene=scene,
            mujoco_root=root,
            elastic_band=robot.elastic_band if band_env is None else _as_bool(band_env),
            world=world,
        )


class ElasticBand:
    """Virtual rope from a hook above the robot to one of its bodies.

    Ported from ``simulate_python/unitree_sdk2py_bridge.py`` so the simulator
    can be driven without the viewer window, with two changes that make
    "lower until the feet touch" behave like a gantry: the rope is ten times
    stiffer (upstream's 200 N/m stretches 1.7 m under a G1, so the robot never
    bears its own weight until released), and it goes slack instead of pushing
    when shorter than its length. The G1 hangs with its pelvis at about 1.28 m;
    each 0.1 m of length lowers it 0.1 m; after seven steps the rope is slack.
    """

    def __init__(self) -> None:
        self.stiffness = 2000.0
        self.damping = 300.0
        self.point = np.array([0.0, 0.0, 1.45])
        self.length = 0.0
        self.enable = True

    def force(self, position: np.ndarray, velocity: np.ndarray) -> np.ndarray:
        offset = self.point - position
        distance = float(np.linalg.norm(offset))
        stretch = distance - self.length
        if distance < 1e-6 or stretch <= 0.0:
            return np.zeros(3)
        direction = offset / distance
        # Damp every direction while taut, not only along the rope; a rope with
        # only axial damping swings as an undamped pendulum, which looks like flight.
        tension = self.stiffness * stretch * direction
        return tension - self.damping * velocity

    def on_key(self, key: int) -> None:
        if key == _KEY_7:
            self.length -= BAND_STEP_M
        elif key == _KEY_8:
            self.length += BAND_STEP_M
        elif key == _KEY_9:
            self.enable = not self.enable

    def apply(self, command: SimCommand) -> None:
        if command.reset:
            self.reset()
            return
        self.enable = bool(command.band_enable)
        self.length += float(command.band_length_delta)

    def reset(self) -> None:
        """Hang the robot again, as at start-up."""
        self.enable = True
        self.length = 0.0


def main() -> None:
    """Start the simulator and block until it is asked to stop."""
    try:
        settings = Settings.from_env()
    except KeyError as exc:
        print(f"{exc} (SETUP-002)", file=sys.stderr)
        raise SystemExit(64) from exc
    if not settings.scene.is_file():
        print(
            f"Scene file not found: {settings.scene}. "
            f"Set UNITREE_ROBOT and UNITREE_MUJOCO_ROOT. (SETUP-001)",
            file=sys.stderr,
        )
        raise SystemExit(66)
    run(settings)


def run(settings: Settings) -> None:
    """Step the physics and publish Unitree DDS state."""
    import mujoco
    import mujoco.viewer
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
    )

    _install_bridge_config(settings.robot.name)
    bridge_dir = settings.mujoco_root / "simulate_python"
    sys.path.insert(0, str(bridge_dir))
    from unitree_sdk2py_bridge import UnitreeSdk2Bridge

    model = build_model(settings.scene, settings.world, LIDAR)
    data = mujoco.MjData(model)
    _reset(mujoco, model, data)
    scan_slice = lidar_sensor_slice(model, LIDAR)
    scan_site = model.site(f"{LIDAR.site_name}_000").id
    scan_body = model.body(LIDAR.body).id
    scan_every = max(1, round(SCAN_DT / model.opt.timestep))

    band = ElasticBand() if settings.elastic_band else None
    band_body = model.body(settings.robot.band_body).id if band is not None else -1
    reset_requested = threading.Event()

    stop = threading.Event()
    _stop_on_signal(stop)
    lock = threading.Lock()
    viewer = None
    if not settings.headless:
        key_callback = band.on_key if band is not None else None
        viewer = mujoco.viewer.launch_passive(model, data, key_callback=key_callback)

    def on_sim_command(command: SimCommand) -> None:
        if band is not None:
            band.apply(command)
            print(
                f"band enable={band.enable} length={band.length:.2f}",
                flush=True,
            )
        if command.reset:
            reset_requested.set()

    class MotorFirmwareBridge(UnitreeSdk2Bridge):  # type: ignore[misc,valid-type]
        """Upstream bridge with the motor PD loop moved into the physics step.

        Upstream turns each ``rt/lowcmd`` into a torque once, on receipt, from
        sensor values that lag a step, and holds it until the next message. A
        real Unitree motor driver instead closes the PD loop at kHz on current
        measurements, so controllers such as ``unitree_rl_gym`` send targets at
        50 Hz. This class keeps the targets and gains and lets ``pd_torque``
        evaluate them every step, which is also far cheaper for the Python
        process than 500 Hz of per-message arithmetic.
        """

        def __init__(self, mj_model: object, mj_data: object) -> None:
            super().__init__(mj_model, mj_data)
            n = self.num_motor
            self.cmd_lock = threading.Lock()
            self.cmd_q = np.zeros(n)
            self.cmd_dq = np.zeros(n)
            self.cmd_tau = np.zeros(n)
            self.cmd_kp = np.zeros(n)
            self.cmd_kd = np.zeros(n)

        def LowCmdHandler(self, msg: object) -> None:  # noqa: N802 - upstream name
            motors = msg.motor_cmd  # type: ignore[attr-defined]
            n = self.num_motor
            with self.cmd_lock:
                self.cmd_q[:] = [motors[i].q for i in range(n)]
                self.cmd_dq[:] = [motors[i].dq for i in range(n)]
                self.cmd_tau[:] = [motors[i].tau for i in range(n)]
                self.cmd_kp[:] = [motors[i].kp for i in range(n)]
                self.cmd_kd[:] = [motors[i].kd for i in range(n)]

        def pd_torque(self) -> np.ndarray:
            """Torque from the latest command and the current joint sensors."""
            n = self.num_motor
            q = data.sensordata[:n]
            dq = data.sensordata[n : 2 * n]
            with self.cmd_lock:
                return (
                    self.cmd_tau + self.cmd_kp * (self.cmd_q - q) + self.cmd_kd * (self.cmd_dq - dq)
                )

    def simulate() -> None:
        ChannelFactoryInitialize(settings.domain_id, settings.interface)
        # Upstream publishes rt/lowstate and rt/sportmodestate once per physics
        # step. 500 Hz of Python serialisation starves the stepping thread, so
        # the bridge is built with upstream's own 5 ms interval and the model's
        # real step is restored afterwards.
        physics_dt = model.opt.timestep
        model.opt.timestep = PUBLISH_DT
        bridge = MotorFirmwareBridge(model, data)
        model.opt.timestep = physics_dt
        command_sub = ChannelSubscriber(TOPIC_SIM_COMMAND, SimCommand)
        command_sub.Init(on_sim_command, 10)
        scan_pub = ChannelPublisher(TOPIC_SCAN, LaserScan)
        scan_pub.Init()
        world_name = settings.world.name if settings.world is not None else DEFAULT_WORLD
        print(
            f"simulator publishing on domain {settings.domain_id} "
            f"interface {settings.interface} robot {settings.robot.name} "
            f"world={world_name} band={'on' if band is not None else 'off'}",
            flush=True,
        )
        steps = 0
        scan = None
        started = time.perf_counter()
        # Absolute schedule so sleep overhead does not slow the physics clock;
        # controllers assume simulated time tracks wall time.
        next_step = started
        report_at = started + RTF_REPORT_S
        sim_time_at_report = data.time
        while not stop.is_set():
            if viewer is not None and not viewer.is_running():
                break
            with lock:
                if reset_requested.is_set():
                    reset_requested.clear()
                    _reset(mujoco, model, data)
                    sim_time_at_report = data.time
                    print("simulation reset", flush=True)
                if band is not None and band.enable:
                    data.xfrc_applied[band_body, :3] = band.force(data.qpos[:3], data.qvel[:3])
                elif band is not None:
                    data.xfrc_applied[band_body, :3] = 0.0
                # step1 brings sensors up to date for the current state; step2 integrates.
                mujoco.mj_step1(model, data)
                data.ctrl[:] = bridge.pd_torque()
                if steps % scan_every == 0:
                    # After step1 the rangefinders describe the current pose. The
                    # rays lie in the lidar body's xy plane, so that body's
                    # orientation at the mount is the scan frame.
                    scan = LaserScan(
                        stamp_ns=time.time_ns(),
                        angle_min=LIDAR.angle_min,
                        angle_increment=LIDAR.angle_increment,
                        range_max=LIDAR.range_max_m,
                        ranges=ranges_from_sensordata(data.sensordata[scan_slice], LIDAR).tolist(),
                        position=data.site_xpos[scan_site].tolist(),
                        orientation=data.xquat[scan_body].tolist(),
                    )
                mujoco.mj_step2(model, data)
                steps += 1
            if scan is not None:
                scan_pub.Write(scan)  # outside the lock: DDS serialisation is not physics
                scan = None
            now = time.perf_counter()
            if settings.duration_s and now - started >= settings.duration_s:
                break
            if now >= report_at:
                factor = (data.time - sim_time_at_report) / RTF_REPORT_S
                print(f"real-time factor {factor:.2f}", flush=True)
                report_at += RTF_REPORT_S
                sim_time_at_report = data.time
            next_step += model.opt.timestep
            remaining = next_step - now
            if remaining > 0:
                time.sleep(remaining)
            elif remaining < -MAX_CATCH_UP_S:
                next_step = now  # too far behind: slow down rather than burst
        stop.set()

    if viewer is None:
        simulate()
        # CycloneDDS threads would otherwise keep the container alive after the run.
        os._exit(0)

    worker = threading.Thread(target=simulate, name="simulate", daemon=True)
    worker.start()
    while viewer.is_running() and not stop.is_set():
        with lock:
            viewer.sync()
        time.sleep(VIEWER_DT)
    stop.set()
    viewer.close()
    worker.join(timeout=2)
    os._exit(0)


def _reset(mujoco: types.ModuleType, model: object, data: object) -> None:
    """Return to the model's home keyframe when it has one, else the initial pose."""
    if model.nkey > 0:  # type: ignore[attr-defined]
        mujoco.mj_resetDataKeyframe(model, data, 0)
    else:
        mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)


def _default_mujoco_root() -> Path:
    return Path(__file__).resolve().parents[2] / "third_party" / "unitree_mujoco"


def _install_bridge_config(robot: str) -> None:
    """Provide the ``config`` module the upstream bridge imports."""
    module = types.ModuleType("config")
    module.ROBOT = robot  # type: ignore[attr-defined]
    sys.modules["config"] = module


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _stop_on_signal(stop: threading.Event) -> None:
    def handler(signum: int, _frame: object) -> None:
        print(f"stopping simulator on signal {signum}", flush=True)
        stop.set()

    signal.signal(signal.SIGTERM, handler)
    signal.signal(signal.SIGINT, handler)
