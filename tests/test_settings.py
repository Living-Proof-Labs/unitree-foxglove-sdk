"""Simulator settings come from the environment and the pinned MuJoCo scene."""

import os
from pathlib import Path

import pytest

from unitree_foxglove.idl import SimCommand
from unitree_foxglove.robots import G1, GO2, profile_for
from unitree_foxglove.sim import BAND_STEP_M, ElasticBand, Settings


def test_default_scene_is_the_g1_29dof_model() -> None:
    settings = Settings.from_env()
    assert settings.robot is G1
    assert settings.domain_id == 1
    assert settings.interface == "lo"
    assert settings.headless is False
    assert settings.elastic_band is True
    assert settings.scene.name == "scene_29dof.xml"
    assert settings.scene.is_file()


def test_go2_scene_has_no_band(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("UNITREE_ROBOT", "go2")
    settings = Settings.from_env()
    assert settings.robot is GO2
    assert settings.elastic_band is False
    assert settings.scene.name == "scene.xml"
    assert settings.scene.is_file()


def test_environment_overrides(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    scene_dir = tmp_path / "unitree_robots" / "g1"
    scene_dir.mkdir(parents=True)
    (scene_dir / "scene_29dof.xml").write_text("<mujoco/>")
    monkeypatch.setenv("UNITREE_DOMAIN_ID", "3")
    monkeypatch.setenv("UNITREE_INTERFACE", "eth0")
    monkeypatch.setenv("UNITREE_HEADLESS", "yes")
    monkeypatch.setenv("UNITREE_DURATION", "4.5")
    monkeypatch.setenv("UNITREE_ELASTIC_BAND", "0")
    monkeypatch.setenv("UNITREE_MUJOCO_ROOT", str(tmp_path))
    monkeypatch.delenv("DISPLAY", raising=False)

    settings = Settings.from_env()
    assert settings.domain_id == 3
    assert settings.interface == "eth0"
    assert settings.headless is True
    assert settings.duration_s == 4.5
    assert settings.elastic_band is False
    assert settings.scene == scene_dir / "scene_29dof.xml"


def test_unknown_robot_lists_the_supported_ones() -> None:
    with pytest.raises(KeyError, match="g1, go2"):
        profile_for("b2")


def test_main_exits_for_unknown_robot_and_missing_scene(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    from unitree_foxglove.sim import main

    monkeypatch.setenv("UNITREE_ROBOT", "missing")
    with pytest.raises(SystemExit) as raised:
        main()
    assert raised.value.code == 64

    monkeypatch.setenv("UNITREE_ROBOT", "g1")
    monkeypatch.setenv("UNITREE_MUJOCO_ROOT", str(tmp_path))
    with pytest.raises(SystemExit) as raised:
        main()
    assert raised.value.code == 66


def test_settings_ignore_unrelated_environment(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("UNITREE_HEADLESS", "0")
    assert os.environ["UNITREE_HEADLESS"] == "0"
    assert Settings.from_env().headless is False


def test_elastic_band_pulls_towards_its_anchor_and_follows_commands() -> None:
    import numpy as np

    band = ElasticBand()
    force = band.force(np.array([0.0, 0.0, 1.0]), np.zeros(3))
    assert force[2] == pytest.approx(2000.0 * 0.45)
    assert force[0] == force[1] == 0.0

    # Damping acts on the whole velocity, so a swinging robot settles instead of
    # orbiting the anchor.
    swinging = band.force(np.array([0.0, 0.0, 1.0]), np.array([0.5, 0.0, 0.0]))
    assert swinging[0] == pytest.approx(-300.0 * 0.5)
    assert swinging[2] == pytest.approx(2000.0 * 0.45)

    band.apply(SimCommand(band_enable=True, band_length_delta=BAND_STEP_M, reset=False))
    assert band.length == pytest.approx(BAND_STEP_M)
    assert band.force(np.array([0.0, 0.0, 1.0]), np.zeros(3))[2] == pytest.approx(2000.0 * 0.35)
    # A rope goes slack instead of pushing.
    assert band.force(np.array([0.0, 0.0, 1.4]), np.zeros(3)) == pytest.approx([0.0, 0.0, 0.0])

    band.apply(SimCommand(band_enable=False, band_length_delta=0.0, reset=False))
    assert band.enable is False

    band.on_key(57)  # viewer key 9 toggles
    assert band.enable is True
    band.on_key(55)  # key 7 shortens
    assert band.length == pytest.approx(0.0)

    band.on_key(56)
    band.on_key(57)
    band.apply(SimCommand(band_enable=False, band_length_delta=0.0, reset=True))
    assert band.enable is True
    assert band.length == 0.0
