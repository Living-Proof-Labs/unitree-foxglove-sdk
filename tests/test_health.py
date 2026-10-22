"""Health classification over synthetic observations; no simulator."""

import math

import pytest

from unitree_foxglove.health import (
    DEGRADED,
    INACTIVE,
    INCIDENT,
    NOMINAL,
    OFFLINE,
    THRESHOLDS,
    HealthMonitor,
    Observation,
    judge,
    tilt_from_quaternion,
)

G1 = THRESHOLDS["g1"]
MS = 1_000_000


def _obs(
    t_ms: int,
    height: float | None = 0.75,
    tilt: float = 2.0,
    mode: str = "walk",
    band: bool = False,
) -> Observation:
    return Observation(t_ms * MS, height, tilt, mode, band)


def test_tilt_from_quaternion() -> None:
    assert tilt_from_quaternion([1.0, 0.0, 0.0, 0.0]) == pytest.approx(0.0)
    # 90 degrees about x, then pure yaw: yaw does not tilt.
    half = math.radians(45)
    assert tilt_from_quaternion([math.cos(half), math.sin(half), 0.0, 0.0]) == pytest.approx(90)
    assert tilt_from_quaternion([math.cos(half), 0.0, 0.0, math.sin(half)]) == pytest.approx(0)


@pytest.mark.parametrize(
    ("observation", "state"),
    [
        (_obs(0), NOMINAL),
        (_obs(0, mode="paused"), NOMINAL),
        (_obs(0, mode="stand"), NOMINAL),
        (_obs(0, height=None), NOMINAL),
        (_obs(0, band=True, height=0.1), INACTIVE),
        (_obs(0, mode="damp", height=0.1), INACTIVE),
        (_obs(0, height=0.30), INCIDENT),
        (_obs(0, tilt=75.0), INCIDENT),
        (_obs(0, height=0.55), DEGRADED),
        (_obs(0, tilt=35.0), DEGRADED),
    ],
)
def test_judge(observation: Observation, state: str) -> None:
    assert judge(observation, G1)[0] == state


def test_a_fall_is_an_incident_only_after_the_hold() -> None:
    monitor = HealthMonitor(G1, 0)
    assert monitor.observe(_obs(0)).state == NOMINAL

    stumble = monitor.observe(_obs(100, height=0.3))
    assert stumble.state == DEGRADED
    assert monitor.observe(_obs(300, height=0.3)) is None
    assert monitor.observe(_obs(400)).state == NOMINAL

    assert monitor.observe(_obs(1000, height=0.3)).state == DEGRADED
    fallen = monitor.observe(_obs(1500, height=0.2))
    assert fallen.state == INCIDENT
    assert fallen.since_ns == 1000 * MS
    assert "0.20 m" in fallen.reason


def test_publishes_on_change_and_once_per_period() -> None:
    monitor = HealthMonitor(G1, 0)
    assert monitor.observe(_obs(0)) is not None
    assert monitor.observe(_obs(500)) is None
    repeat = monitor.observe(_obs(1000))
    assert repeat is not None and repeat.state == NOMINAL
    assert repeat.since_ns == 0
    assert monitor.observe(_obs(1100, band=True)).state == INACTIVE


def test_offline_after_a_second_without_state() -> None:
    monitor = HealthMonitor(G1, 0)
    assert monitor.tick(500 * MS) is None
    assert monitor.tick(1000 * MS).state == OFFLINE
    monitor.observe(_obs(1200))
    assert monitor.current.state == NOMINAL
    assert monitor.tick(1500 * MS) is None
    assert monitor.tick(2300 * MS).state == OFFLINE


def test_every_profile_has_thresholds() -> None:
    from unitree_foxglove.robots import PROFILES

    assert set(PROFILES) <= set(THRESHOLDS)
