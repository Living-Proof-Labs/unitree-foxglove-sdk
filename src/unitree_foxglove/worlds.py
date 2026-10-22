"""Worlds the simulator can load around the upstream robot scene, and the lidar.

The upstream ``scene_*.xml`` is an open floor. A world adds walls to it at load
time through ``MjSpec``, so ``third_party/`` stays untouched. The same
description is what the Foxglove bridge draws as the static scene, so the
simulator and the 3D panel agree on where the walls are.

The lidar is a fan of MuJoCo rangefinders on the torso, above the head. It
stands in for the Livox on a real G1's head and produces one planar scan.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

WALL_THICKNESS_M = 0.1


@dataclass(frozen=True)
class Box:
    """An axis-aligned box: centre and half sizes, in metres."""

    name: str
    center: tuple[float, float, float]
    half: tuple[float, float, float]


@dataclass(frozen=True)
class World:
    """A straight hallway along +x, closed at both ends.

    The robot starts at the origin facing +x. The near wall is ``start_m``
    behind it; the far wall is ``length_m - start_m`` ahead.
    """

    name: str
    length_m: float
    width_m: float = 2.0
    wall_height_m: float = 2.0
    start_m: float = 1.0

    @property
    def far_wall_x(self) -> float:
        return self.length_m - self.start_m

    @property
    def near_wall_x(self) -> float:
        return -self.start_m

    def boxes(self) -> list[Box]:
        half_len = self.length_m / 2 + WALL_THICKNESS_M
        half_w = self.width_m / 2
        half_h = self.wall_height_m / 2
        mid_x = (self.near_wall_x + self.far_wall_x) / 2
        t = WALL_THICKNESS_M / 2
        return [
            Box("wall_left", (mid_x, half_w + t, half_h), (half_len, t, half_h)),
            Box("wall_right", (mid_x, -half_w - t, half_h), (half_len, t, half_h)),
            Box("wall_far", (self.far_wall_x + t, 0.0, half_h), (t, half_w, half_h)),
            Box("wall_near", (self.near_wall_x - t, 0.0, half_h), (t, half_w, half_h)),
        ]


WORLDS: dict[str, World | None] = {
    "open": None,
    "hallway10": World("hallway10", length_m=10.0),
    "hallway20": World("hallway20", length_m=20.0),
}
DEFAULT_WORLD = "open"


def world_for(name: str) -> World | None:
    """Look up a world; ``None`` is the open floor."""
    try:
        return WORLDS[name]
    except KeyError:
        supported = ", ".join(sorted(WORLDS))
        raise KeyError(f"Unknown world {name!r}. Supported: {supported}") from None


@dataclass(frozen=True)
class Lidar:
    """Planar fan of rangefinders. Angles are counter-clockwise from +x."""

    body: str = "torso_link"
    mount: tuple[float, float, float] = (0.0, 0.0, 0.5)
    """Site position in the body frame; 0.5 m above the torso origin clears the head."""
    rays: int = 72
    range_max_m: float = 10.0
    frame_id: str = "lidar"
    site_name: str = "lidar"

    @property
    def angle_increment(self) -> float:
        return 2 * math.pi / self.rays

    @property
    def angle_min(self) -> float:
        return -math.pi

    def angles(self) -> np.ndarray:
        return self.angle_min + self.angle_increment * np.arange(self.rays)

    def sensor_name(self, index: int) -> str:
        return f"lidar_{index:03d}"


LIDAR = Lidar()


def build_model(scene: Path, world: World | None, lidar: Lidar | None = LIDAR) -> object:
    """Compile the upstream scene with the world's walls and the lidar added.

    Rangefinders are appended after upstream's sensors, so the joint, IMU, and
    frame indices the upstream DDS bridge reads from ``sensordata`` do not move.
    Returns an ``mujoco.MjModel``.
    """
    import mujoco

    spec = mujoco.MjSpec.from_file(str(scene))
    if world is not None:
        for box in world.boxes():
            geom = spec.worldbody.add_geom()
            geom.name = box.name
            geom.type = mujoco.mjtGeom.mjGEOM_BOX
            geom.pos[:] = box.center
            geom.size[:] = box.half
            geom.rgba[:] = (0.75, 0.72, 0.65, 1.0)
    if lidar is not None:
        body = spec.body(lidar.body)
        for index, angle in enumerate(lidar.angles()):
            site = body.add_site()
            site.name = f"{lidar.site_name}_{index:03d}"
            site.pos[:] = lidar.mount
            site.quat[:] = _ray_quat(float(angle))
            site.size[:] = (0.005, 0.005, 0.005)
            site.group = 4  # hidden by default in the viewer
            sensor = spec.add_sensor()
            sensor.name = lidar.sensor_name(index)
            sensor.type = mujoco.mjtSensor.mjSENS_RANGEFINDER
            sensor.objtype = mujoco.mjtObj.mjOBJ_SITE
            sensor.objname = site.name
            sensor.cutoff = lidar.range_max_m
            sensor.intprm[0] = 1  # one ray per sensor; MuJoCo 3.14 defaults to a 0-ray grid
    return spec.compile()


def _ray_quat(yaw: float) -> tuple[float, float, float, float]:
    """Quaternion (w, x, y, z) whose +z axis points along ``yaw`` in the body xy plane.

    A rangefinder measures along its site's +z. Rotate z onto x (about y by
    +90 degrees), then yaw about z.
    """
    half_pitch = math.pi / 4
    q_pitch = (math.cos(half_pitch), 0.0, math.sin(half_pitch), 0.0)
    q_yaw = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
    return _quat_mul(q_yaw, q_pitch)


def _quat_mul(a: tuple[float, ...], b: tuple[float, ...]) -> tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def lidar_sensor_slice(model: object, lidar: Lidar) -> slice:
    """Where the fan's ranges sit in ``data.sensordata``."""
    first = model.sensor(lidar.sensor_name(0)).adr[0]  # type: ignore[attr-defined]
    return slice(int(first), int(first) + lidar.rays)


def ranges_from_sensordata(values: np.ndarray, lidar: Lidar) -> np.ndarray:
    """Rangefinder returns -1 when nothing is within the cutoff; report that as inf."""
    ranges = np.asarray(values, dtype=np.float32).copy()
    ranges[ranges < 0] = np.inf
    ranges[ranges > lidar.range_max_m] = np.inf
    return ranges
