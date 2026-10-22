"""Robot profiles: which DDS types, scene, and joint names a robot uses.

Quadrupeds (Go2, B2) speak ``unitree_go`` messages with 20 motor slots. Humanoids
(G1, H1-2) speak ``unitree_hg`` messages with 35 motor slots. The MuJoCo bridge
picks ``unitree_hg`` when the model has more than 20 actuators, so the profile
here has to agree with the scene it names.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class RobotProfile:
    """Static facts about one robot the simulator can load."""

    name: str
    idl: str
    """``"go"`` for unitree_go messages, ``"hg"`` for unitree_hg messages."""
    scene_file: str
    joint_names: tuple[str, ...]
    elastic_band: bool
    """Hang the robot from a virtual spring at start-up (humanoids)."""
    band_body: str
    """MuJoCo body the band pulls on."""
    base_frame: str
    """Link the IMU pose refers to; child of ``world`` in the transform tree."""
    description_dir: str | None = None
    """Directory under ``unitree_rl_gym/resources/robots`` with the URDF and meshes."""
    urdf_file: str | None = None
    """URDF inside ``description_dir`` whose joints match ``joint_names``."""

    @property
    def num_joints(self) -> int:
        return len(self.joint_names)

    def low_state_type(self) -> type:
        if self.idl == "hg":
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

            return LowState_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

        return LowState_

    def low_cmd_type(self) -> type:
        if self.idl == "hg":
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

            return LowCmd_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_

        return LowCmd_

    def description_path(self, rl_gym_root: Path) -> Path | None:
        """Directory holding the URDF and meshes, or ``None`` when the robot has none."""
        if self.description_dir is None:
            return None
        return rl_gym_root / "resources" / "robots" / self.description_dir

    def new_low_cmd(self) -> object:
        if self.idl == "hg":
            from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_

            return unitree_hg_msg_dds__LowCmd_()
        from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_

        return unitree_go_msg_dds__LowCmd_()


# Sensor order in third_party/unitree_mujoco/unitree_robots/go2/go2.xml.
GO2 = RobotProfile(
    name="go2",
    idl="go",
    scene_file="scene.xml",
    joint_names=(
        "FR_hip_joint",
        "FR_thigh_joint",
        "FR_calf_joint",
        "FL_hip_joint",
        "FL_thigh_joint",
        "FL_calf_joint",
        "RR_hip_joint",
        "RR_thigh_joint",
        "RR_calf_joint",
        "RL_hip_joint",
        "RL_thigh_joint",
        "RL_calf_joint",
    ),
    elastic_band=False,
    band_body="base_link",
    base_frame="base_link",
)

# Motor order in third_party/unitree_mujoco/unitree_robots/g1/g1_29dof.xml.
# Matches the G1 EDU with the 3-dof waist and 7-dof arms.
G1 = RobotProfile(
    name="g1",
    idl="hg",
    scene_file="scene_29dof.xml",
    joint_names=(
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        "waist_yaw_joint",
        "waist_roll_joint",
        "waist_pitch_joint",
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ),
    elastic_band=True,
    band_body="torso_link",
    base_frame="pelvis",
    description_dir="g1_description",
    urdf_file="g1_29dof.urdf",
)

PROFILES: dict[str, RobotProfile] = {GO2.name: GO2, G1.name: G1}

DEFAULT_ROBOT = G1.name


def profile_for(name: str) -> RobotProfile:
    """Look up a profile, raising ``KeyError`` with the supported names."""
    try:
        return PROFILES[name]
    except KeyError:
        supported = ", ".join(sorted(PROFILES))
        raise KeyError(f"Unknown robot {name!r}. Supported: {supported}") from None
