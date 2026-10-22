"""Observation layout and command handling of the G1 walking controller.

Uses the real ``g1.yaml`` from the submodule but never loads torch.
"""

import numpy as np
import pytest

from unitree_foxglove.loco import (
    COMMAND_TIMEOUT_S,
    CommandState,
    PolicyConfig,
    build_observation,
    clamp_command,
    default_paths,
    gravity_orientation,
)


@pytest.fixture(scope="module")
def config() -> PolicyConfig:
    config_path, policy_path = default_paths()
    if not config_path.is_file():
        pytest.skip("unitree_rl_gym submodule not checked out")
    return PolicyConfig.load(config_path, policy_path)


def test_config_matches_the_upstream_g1_deployment(config: PolicyConfig) -> None:
    assert config.control_dt == 0.02
    assert config.num_obs == 47
    assert config.num_actions == 12
    assert config.leg_joint2motor_idx == list(range(12))
    assert config.arm_waist_joint2motor_idx == list(range(12, 29))
    assert len(config.kps) == len(config.kds) == 12
    assert len(config.arm_waist_kps) == len(config.arm_waist_target) == 17
    assert config.default_angles.shape == (12,)
    assert list(config.max_cmd) == pytest.approx([0.8, 0.5, 1.57])


def test_observation_layout_is_gyro_gravity_cmd_pos_vel_action_phase(
    config: PolicyConfig,
) -> None:
    joint_pos = config.default_angles + 0.1
    joint_vel = np.full(12, 2.0, dtype=np.float32)
    last_action = np.arange(12, dtype=np.float32)
    obs = build_observation(
        config,
        joint_pos=joint_pos,
        joint_vel=joint_vel,
        quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        gyroscope=np.array([0.4, 0.0, -0.4]),
        command=np.array([0.5, 0.0, 0.0], dtype=np.float32),
        last_action=last_action,
        step=0,
    )
    assert obs.shape == (47,)
    assert obs.dtype == np.float32
    assert obs[0:3] == pytest.approx([0.1, 0.0, -0.1])  # gyro * 0.25
    assert obs[3:6] == pytest.approx([0.0, 0.0, -1.0])  # upright: gravity is -z
    assert obs[6:9] == pytest.approx([1.0, 0.0, 0.0])  # vx * cmd_scale 2
    assert obs[9:21] == pytest.approx(np.full(12, 0.1))  # (q - default) * 1
    assert obs[21:33] == pytest.approx(np.full(12, 0.1))  # dq * 0.05
    assert obs[33:45] == pytest.approx(last_action)
    assert obs[45:47] == pytest.approx([0.0, 1.0])  # sin/cos of phase 0


def test_gait_phase_advances_with_the_step_counter(config: PolicyConfig) -> None:
    zeros = np.zeros(12, dtype=np.float32)

    def phase(step: int) -> tuple[float, float]:
        obs = build_observation(
            config,
            joint_pos=config.default_angles,
            joint_vel=zeros,
            quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
            gyroscope=np.zeros(3),
            command=np.zeros(3, dtype=np.float32),
            last_action=zeros,
            step=step,
        )
        return float(obs[45]), float(obs[46])

    # 0.8 s period at 50 Hz is 40 steps; a quarter period is 10 steps.
    assert phase(10) == pytest.approx((1.0, 0.0), abs=1e-6)
    assert phase(40) == pytest.approx((0.0, 1.0), abs=1e-6)


def test_gravity_orientation_for_a_tilted_body() -> None:
    # 90 degrees about y: the body x axis points down.
    half = np.sqrt(0.5)
    gravity = gravity_orientation(np.array([half, 0.0, half, 0.0]))
    assert gravity == pytest.approx([1.0, 0.0, 0.0], abs=1e-6)


def test_commands_are_clamped_to_the_training_range(config: PolicyConfig) -> None:
    assert clamp_command(config, 2.0, -3.0, 5.0) == pytest.approx([0.8, -0.5, 1.57])
    assert clamp_command(config, 0.3, 0.1, -0.2) == pytest.approx([0.3, 0.1, -0.2])


def test_stale_velocity_command_becomes_zero() -> None:
    state = CommandState(velocity=np.array([0.4, 0.0, 0.0], dtype=np.float32), received_at=100.0)
    assert state.current_velocity(100.2) == pytest.approx([0.4, 0.0, 0.0])
    assert state.current_velocity(100.0 + COMMAND_TIMEOUT_S + 0.01) == pytest.approx([0, 0, 0])
