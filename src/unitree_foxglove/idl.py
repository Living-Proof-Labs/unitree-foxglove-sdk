"""DDS messages exchanged between this project's own processes.

The Unitree topics (``rt/lowstate``, ``rt/lowcmd``) stay as upstream defines
them. ``LocoCommand`` and ``SimCommand`` carry what the simulator and the walking
controller need from the Foxglove bridge and would have no equivalent on a real
robot, where the onboard ``LocoClient`` takes the same commands. ``LaserScan``
is what a lidar driver on the real robot would publish; the simulator publishes
it from a fan of rangefinders.

No ``from __future__ import annotations`` here: CycloneDDS reads the real
annotation objects to build the type descriptor.
"""

from dataclasses import dataclass

from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import array, float32, float64, int64, sequence

TOPIC_LOCO_COMMAND = "loco/command"
TOPIC_SIM_COMMAND = "sim/command"
TOPIC_SCAN = "sensor/scan"

MODE_DAMP = "damp"
MODE_STAND = "stand"
MODE_WALK = "walk"
MODES = (MODE_DAMP, MODE_STAND, MODE_WALK)


@dataclass
class LocoCommand(IdlStruct, typename="unitree_foxglove.LocoCommand"):
    """Desired controller mode and body velocity."""

    mode: str
    vx: float64
    vy: float64
    wz: float64
    stamp_ns: int64


@dataclass
class SimCommand(IdlStruct, typename="unitree_foxglove.SimCommand"):
    """Elastic band and reset requests for the simulator."""

    band_enable: bool
    band_length_delta: float64
    reset: bool


@dataclass
class LaserScan(IdlStruct, typename="unitree_foxglove.LaserScan"):
    """One planar scan plus the sensor pose in the world frame.

    Angles are counter-clockwise from the sensor's +x; ``ranges[i]`` is at
    ``angle_min + i * angle_increment``. A range of ``inf`` means no return
    within ``range_max``. ``position`` and ``orientation`` (w, x, y, z) place the
    sensor in the world so consumers need no robot model to draw the scan.
    """

    stamp_ns: int64
    angle_min: float64
    angle_increment: float64
    range_max: float64
    ranges: sequence[float32]
    position: array[float64, 3]
    orientation: array[float64, 4]
