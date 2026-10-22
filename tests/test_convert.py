"""DDS sample to Foxglove message conversion, without a simulator."""

from types import SimpleNamespace

import pytest

from unitree_foxglove.check_mcap import decode_payload
from unitree_foxglove.convert import (
    base_pose_from_state,
    base_transform_from_state,
    base_velocity_from_sport,
    imu_from_lowstate,
    joint_states_from_lowstate,
    laser_scan_from_dds,
    scene_update_from_world,
    timestamp_from_ns,
    twist_from_json,
)
from unitree_foxglove.robots import G1, GO2
from unitree_foxglove.worlds import WORLDS


def _motor(position: float, velocity: float, effort: float) -> SimpleNamespace:
    return SimpleNamespace(q=position, dq=velocity, tau_est=effort)


def _low_state(slots: int) -> SimpleNamespace:
    motors = [_motor(0.1 * index, 0.01 * index, 0.2) for index in range(slots)]
    imu = SimpleNamespace(
        quaternion=[0.0, 1.0, 0.0, 0.0],
        gyroscope=[0.1, 0.2, 0.3],
        accelerometer=[0.0, 0.0, 9.81],
    )
    return SimpleNamespace(motor_state=motors, imu_state=imu)


def _sport_state() -> SimpleNamespace:
    return SimpleNamespace(position=[1.0, 2.0, 0.3], velocity=[0.4, -0.1, 0.0])


def test_timestamp_splits_nanoseconds() -> None:
    stamp = timestamp_from_ns(1_500_000_003)
    assert stamp.sec == 1
    assert stamp.nsec == 500_000_003


def _fields(message: object) -> object:
    schema = message.get_schema()  # type: ignore[attr-defined]
    return decode_payload(schema.name, schema.data, message.encode())  # type: ignore[attr-defined]


def test_joint_states_follow_go2_sensor_order() -> None:
    message = _fields(joint_states_from_lowstate(_low_state(20), timestamp_from_ns(0), GO2))
    assert [joint.name for joint in message.joints] == list(GO2.joint_names)
    assert message.joints[0].position == 0.0
    assert message.joints[3].name == "FL_hip_joint"
    assert message.joints[3].velocity == 0.03
    assert message.joints[11].effort == 0.2


def test_joint_states_take_29_of_the_35_hg_motor_slots() -> None:
    message = _fields(joint_states_from_lowstate(_low_state(35), timestamp_from_ns(0), G1))
    assert len(message.joints) == 29
    assert [joint.name for joint in message.joints] == list(G1.joint_names)
    assert message.joints[12].name == "waist_yaw_joint"
    assert message.joints[28].name == "right_wrist_yaw_joint"
    assert message.joints[28].position == pytest.approx(2.8)


def test_base_pose_swaps_quaternion_to_xyzw() -> None:
    pose = _fields(
        base_pose_from_state(_low_state(35), _sport_state(), timestamp_from_ns(2_000_000_000))
    )
    assert pose.frame_id == "world"
    assert pose.pose.position.x == 1.0
    assert pose.pose.position.y == 2.0
    assert pose.pose.position.z == 0.3
    # Source quaternion is w, x, y, z = 0, 1, 0, 0.
    assert pose.pose.orientation.w == 0.0
    assert pose.pose.orientation.x == 1.0
    assert pose.pose.orientation.y == 0.0
    assert pose.pose.orientation.z == 0.0
    assert pose.timestamp.seconds == 2


def test_base_transform_places_the_urdf_root_under_world() -> None:
    message = _fields(
        base_transform_from_state(_low_state(35), _sport_state(), timestamp_from_ns(0), G1)
    )
    assert len(message.transforms) == 1
    transform = message.transforms[0]
    assert transform.parent_frame_id == "world"
    assert transform.child_frame_id == "pelvis"
    assert transform.translation.z == pytest.approx(0.3)
    assert transform.rotation.x == 1.0
    assert transform.rotation.w == 0.0


def test_velocity_and_imu_vectors() -> None:
    velocity = _fields(base_velocity_from_sport(_sport_state()))
    gyro, accel = imu_from_lowstate(_low_state(35))
    gyro, accel = _fields(gyro), _fields(accel)
    assert (velocity.x, velocity.y, velocity.z) == (0.4, -0.1, 0.0)
    assert (gyro.x, gyro.y, gyro.z) == (0.1, 0.2, 0.3)
    assert accel.z == 9.81


def test_twist_from_teleop_panel_json() -> None:
    teleop = {"linear": {"x": 0.4, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": -0.6}}
    assert twist_from_json(teleop) == (0.4, 0.0, -0.6)
    assert twist_from_json({}) == (0.0, 0.0, 0.0)
    assert twist_from_json({"linear": None, "angular": {"z": "0.5"}}) == (0.0, 0.0, 0.5)


def test_laser_scan_is_posed_in_the_world_frame() -> None:
    sample = SimpleNamespace(
        angle_min=-3.0,
        angle_increment=1.0,
        range_max=10.0,
        ranges=[1.0, 2.0, float("inf"), 4.0],
        position=[0.5, -0.2, 1.3],
        orientation=[0.0, 0.0, 0.0, 1.0],  # w, x, y, z: 180 degrees about z
    )
    message = _fields(laser_scan_from_dds(sample, timestamp_from_ns(5)))
    assert message.frame_id == "world"
    assert message.start_angle == pytest.approx(-3.0)
    assert message.end_angle == pytest.approx(0.0)
    assert list(message.ranges)[:2] == [1.0, 2.0]
    assert message.pose.position.x == pytest.approx(0.5)
    assert message.pose.position.z == pytest.approx(1.3)
    assert (message.pose.orientation.w, message.pose.orientation.z) == (0.0, 1.0)


def test_scene_update_draws_every_wall() -> None:
    world = WORLDS["hallway20"]
    assert world is not None
    update = _fields(scene_update_from_world(world, timestamp_from_ns(0)))
    (entity,) = update.entities
    assert entity.id == "hallway20"
    assert entity.frame_id == "world"
    assert len(entity.cubes) == 4
    cubes = dict(zip((box.name for box in world.boxes()), entity.cubes, strict=True))
    far = cubes["wall_far"]
    assert far.pose.position.x == pytest.approx(world.far_wall_x + 0.05)
    assert far.size.y == pytest.approx(world.width_m)
