"""Map Unitree DDS samples onto typed Foxglove messages.

``rt/lowstate.motor_state`` follows the actuator order of the MuJoCo model, which
each :class:`~unitree_foxglove.robots.RobotProfile` lists by name. The IMU
quaternion on that topic is w, x, y, z.
"""

from __future__ import annotations

from foxglove.messages import (
    Color,
    CubePrimitive,
    FrameTransform,
    FrameTransforms,
    JointState,
    JointStates,
    LaserScan,
    Pose,
    PoseInFrame,
    Quaternion,
    SceneEntity,
    SceneUpdate,
    Timestamp,
    Vector3,
)

from unitree_foxglove.robots import RobotProfile
from unitree_foxglove.worlds import World

TOPIC_JOINT_STATES = "/robot/joint_states"
TOPIC_BASE_POSE = "/robot/base_pose"
TOPIC_BASE_VELOCITY = "/robot/base_velocity"
TOPIC_IMU_GYRO = "/robot/imu_gyro"
TOPIC_IMU_ACCEL = "/robot/imu_accel"
TOPIC_TF = "/tf"
TOPIC_CMD_VEL = "/cmd_vel"
TOPIC_LOCO_MODE = "/loco/mode"
TOPIC_SCAN = "/scan"
TOPIC_WORLD_SCENE = "/world/scene"

JOINT_TOPICS = (TOPIC_JOINT_STATES,)
POSE_TOPICS = (TOPIC_BASE_POSE, TOPIC_BASE_VELOCITY, TOPIC_TF)
IMU_TOPICS = (TOPIC_IMU_GYRO, TOPIC_IMU_ACCEL)

WALL_COLOR = Color(r=0.75, g=0.72, b=0.65, a=0.6)


def timestamp_from_ns(nanoseconds: int) -> Timestamp:
    """Split a Unix timestamp in nanoseconds into a Foxglove Timestamp."""
    seconds, remainder = divmod(int(nanoseconds), 1_000_000_000)
    return Timestamp(seconds, remainder)


def joint_states_from_lowstate(
    low_state: object, timestamp: Timestamp, profile: RobotProfile
) -> JointStates:
    """Build joint states for the motors the profile names."""
    motors = low_state.motor_state  # type: ignore[attr-defined]
    joints = []
    for index, name in enumerate(profile.joint_names):
        motor = motors[index]
        joints.append(
            JointState(
                name=name,
                position=float(motor.q),
                velocity=float(motor.dq),
                effort=float(motor.tau_est),
            )
        )
    return JointStates(timestamp=timestamp, joints=joints)


def base_pose_from_state(
    low_state: object, sport_state: object, timestamp: Timestamp
) -> PoseInFrame:
    """Pose of the body. Orientation comes from the low-state IMU quaternion."""
    position, orientation = _base_position_and_orientation(low_state, sport_state)
    return PoseInFrame(
        timestamp=timestamp,
        frame_id="world",
        pose=Pose(position=position, orientation=orientation),
    )


def _base_position_and_orientation(
    low_state: object, sport_state: object
) -> tuple[Vector3, Quaternion]:
    quaternion = low_state.imu_state.quaternion  # type: ignore[attr-defined]
    position = sport_state.position  # type: ignore[attr-defined]
    return (
        Vector3(x=float(position[0]), y=float(position[1]), z=float(position[2])),
        # Unitree stores w, x, y, z. Foxglove stores x, y, z, w.
        Quaternion(
            x=float(quaternion[1]),
            y=float(quaternion[2]),
            z=float(quaternion[3]),
            w=float(quaternion[0]),
        ),
    )


def base_transform_from_state(
    low_state: object, sport_state: object, timestamp: Timestamp, profile: RobotProfile
) -> FrameTransforms:
    """``world -> base link`` transform so the 3D panel can place the URDF.

    Same data as :func:`base_pose_from_state`; the URDF layer poses the rest
    of the links from ``/robot/joint_states``.
    """
    position, orientation = _base_position_and_orientation(low_state, sport_state)
    return FrameTransforms(
        transforms=[
            FrameTransform(
                timestamp=timestamp,
                parent_frame_id="world",
                child_frame_id=profile.base_frame,
                translation=position,
                rotation=orientation,
            )
        ]
    )


def base_velocity_from_sport(sport_state: object) -> Vector3:
    """Linear velocity from ``rt/sportmodestate``. PoseInFrame has no velocity field."""
    velocity = sport_state.velocity  # type: ignore[attr-defined]
    return Vector3(x=float(velocity[0]), y=float(velocity[1]), z=float(velocity[2]))


def imu_from_lowstate(low_state: object) -> tuple[Vector3, Vector3]:
    """Gyroscope (rad/s) and accelerometer (m/s^2) from ``rt/lowstate``."""
    gyroscope = low_state.imu_state.gyroscope  # type: ignore[attr-defined]
    accelerometer = low_state.imu_state.accelerometer  # type: ignore[attr-defined]
    gyro = Vector3(x=float(gyroscope[0]), y=float(gyroscope[1]), z=float(gyroscope[2]))
    accel = Vector3(
        x=float(accelerometer[0]),
        y=float(accelerometer[1]),
        z=float(accelerometer[2]),
    )
    return gyro, accel


def laser_scan_from_dds(scan: object, timestamp: Timestamp) -> LaserScan:
    """The simulator's ``sensor/scan`` as a Foxglove LaserScan posed in ``world``.

    The DDS message carries the sensor pose, so the 3D panel needs no
    transform tree to place the points; the scan is drawn where the walls are.
    """
    position = scan.position  # type: ignore[attr-defined]
    orientation = scan.orientation  # type: ignore[attr-defined]  # w, x, y, z
    count = len(scan.ranges)  # type: ignore[attr-defined]
    start = float(scan.angle_min)  # type: ignore[attr-defined]
    end = start + float(scan.angle_increment) * max(count - 1, 0)  # type: ignore[attr-defined]
    return LaserScan(
        timestamp=timestamp,
        frame_id="world",
        pose=Pose(
            position=Vector3(x=float(position[0]), y=float(position[1]), z=float(position[2])),
            orientation=Quaternion(
                x=float(orientation[1]),
                y=float(orientation[2]),
                z=float(orientation[3]),
                w=float(orientation[0]),
            ),
        ),
        start_angle=start,
        end_angle=end,
        ranges=[float(r) for r in scan.ranges],  # type: ignore[attr-defined]
        intensities=[],
    )


def scene_update_from_world(world: World, timestamp: Timestamp) -> SceneUpdate:
    """The world's walls as translucent cubes in the ``world`` frame."""
    cubes = [
        CubePrimitive(
            pose=Pose(
                position=Vector3(x=box.center[0], y=box.center[1], z=box.center[2]),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            size=Vector3(x=2 * box.half[0], y=2 * box.half[1], z=2 * box.half[2]),
            color=WALL_COLOR,
        )
        for box in world.boxes()
    ]
    return SceneUpdate(
        entities=[
            SceneEntity(
                timestamp=timestamp,
                frame_id="world",
                id=world.name,
                frame_locked=True,
                cubes=cubes,
            )
        ]
    )


def twist_from_json(payload: dict) -> tuple[float, float, float]:
    """Read ``linear.x``, ``linear.y``, ``angular.z`` from a ``geometry_msgs/Twist``."""
    linear = payload.get("linear") or {}
    angular = payload.get("angular") or {}
    return (
        float(linear.get("x", 0.0)),
        float(linear.get("y", 0.0)),
        float(angular.get("z", 0.0)),
    )
