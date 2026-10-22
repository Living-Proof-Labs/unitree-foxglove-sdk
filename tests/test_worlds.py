"""Hallway worlds compile onto the upstream scene without moving its sensors."""

from pathlib import Path

import mujoco
import numpy as np
import pytest

from unitree_foxglove.worlds import (
    LIDAR,
    WORLDS,
    build_model,
    lidar_sensor_slice,
    ranges_from_sensordata,
    world_for,
)

SCENE = (
    Path(__file__).resolve().parents[1]
    / "third_party/unitree_mujoco/unitree_robots/g1/scene_29dof.xml"
)


def test_worlds_are_closed_hallways_around_the_origin() -> None:
    for name in ("hallway10", "hallway20"):
        world = world_for(name)
        assert world is not None
        assert world.far_wall_x - world.near_wall_x == pytest.approx(world.length_m)
        assert world.near_wall_x < 0 < world.far_wall_x
        assert {box.name for box in world.boxes()} == {
            "wall_left",
            "wall_right",
            "wall_far",
            "wall_near",
        }
    assert world_for("open") is None
    with pytest.raises(KeyError, match="Supported"):
        world_for("maze")


@pytest.mark.skipif(not SCENE.is_file(), reason="upstream submodule not checked out")
def test_hallway_model_keeps_upstream_sensor_layout_and_adds_the_fan() -> None:
    plain = mujoco.MjModel.from_xml_path(str(SCENE))
    model = build_model(SCENE, world_for("hallway10"))
    # Upstream's bridge indexes joints, IMU, and frame sensors from the front.
    for index in range(plain.nsensor):
        assert model.sensor(index).name == plain.sensor(index).name
        assert model.sensor_adr[index] == plain.sensor_adr[index]
    assert model.nsensor == plain.nsensor + LIDAR.rays
    assert model.ngeom == plain.ngeom + 4
    assert np.array_equal(model.body_mass, plain.body_mass)

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    ranges = ranges_from_sensordata(data.sensordata[lidar_sensor_slice(model, LIDAR)], LIDAR)
    angles = np.degrees(LIDAR.angles())
    by_angle = dict(zip(np.round(angles).astype(int), ranges, strict=True))
    world = WORLDS["hallway10"]
    assert world is not None
    assert by_angle[0] == pytest.approx(world.far_wall_x, abs=0.05)
    assert by_angle[-180] == pytest.approx(-world.near_wall_x, abs=0.05)
    assert by_angle[90] == pytest.approx(world.width_m / 2, abs=0.05)
    assert by_angle[-90] == pytest.approx(world.width_m / 2, abs=0.05)


@pytest.mark.skipif(not SCENE.is_file(), reason="upstream submodule not checked out")
def test_open_world_fan_sees_nothing() -> None:
    model = build_model(SCENE, None)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    ranges = ranges_from_sensordata(data.sensordata[lidar_sensor_slice(model, LIDAR)], LIDAR)
    assert np.all(np.isinf(ranges))


def test_ranges_map_no_return_to_inf() -> None:
    ranges = ranges_from_sensordata(np.array([-1.0, 2.5, 10.5]), LIDAR)
    assert np.isinf(ranges[0]) and np.isinf(ranges[2])
    assert ranges[1] == pytest.approx(2.5)
