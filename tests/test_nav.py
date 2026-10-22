"""The hallway policy on synthetic scans: centring, alignment, arrival, and cancellation."""

import math

import numpy as np
import pytest

from unitree_foxglove.idl import LaserScan
from unitree_foxglove.nav import (
    PARAMS,
    STATE_ALIGNING,
    STATE_ARRIVED,
    STATE_BLIND,
    STATE_STOPPED,
    STATE_WALKING,
    NavController,
    Scan,
    hallway_command,
)

RAYS = 72
STEP = 2 * math.pi / RAYS


def synthetic(
    x: float = 0.0,
    y: float = 0.0,
    yaw_deg: float = 0.0,
    far: float = 9.0,
    near: float = -1.0,
    width: float = 2.0,
    range_max: float = 10.0,
) -> list[float]:
    """Ranges from a robot at (x, y) with heading ``yaw_deg`` in a closed hallway along +x."""
    ranges = []
    for index in range(RAYS):
        bearing = -math.pi + STEP * index + math.radians(yaw_deg)
        dx, dy = math.cos(bearing), math.sin(bearing)
        hits = []
        for wall_y in (width / 2, -width / 2):
            if abs(dy) > 1e-9 and (t := (wall_y - y) / dy) > 0:
                hits.append(t)
        for wall_x in (far, near):
            if abs(dx) > 1e-9 and (t := (wall_x - x) / dx) > 0:
                hits.append(t)
        r = min(hits)
        ranges.append(r if r <= range_max else math.inf)
    return ranges


def scan(**kwargs) -> Scan:
    return Scan(angles=-math.pi + STEP * np.arange(RAYS), ranges=np.array(synthetic(**kwargs)))


def test_centred_and_aligned_walks_straight() -> None:
    decision, direction = hallway_command(scan())
    assert decision.state == STATE_WALKING
    assert decision.vx == pytest.approx(PARAMS.cruise_speed)
    assert decision.wz == pytest.approx(0.0, abs=1e-6)
    assert decision.left_m == pytest.approx(1.0, abs=0.02)
    assert decision.right_m == pytest.approx(1.0, abs=0.02)
    assert decision.front_m == pytest.approx(9.0, abs=0.02)
    assert direction is not None and direction[0] > 0.99


def test_offset_steers_back_to_the_centre_line() -> None:
    left, _ = hallway_command(scan(y=0.3))
    right, _ = hallway_command(scan(y=-0.3))
    assert left.wz < 0 < right.wz  # too far left: turn right, and vice versa
    assert left.left_m == pytest.approx(0.7, abs=0.02)
    assert left.right_m == pytest.approx(1.3, abs=0.02)


def test_heading_error_is_corrected_and_large_errors_turn_in_place() -> None:
    small, _ = hallway_command(scan(yaw_deg=10))
    assert small.state == STATE_WALKING
    assert small.heading_deg == pytest.approx(-10, abs=1)
    assert small.wz < 0
    large, _ = hallway_command(scan(yaw_deg=-120, x=-0.5, y=-0.25))
    assert large.state == STATE_ALIGNING
    assert large.vx == 0.0
    assert large.heading_deg == pytest.approx(120, abs=2)
    assert large.wz == pytest.approx(PARAMS.max_yaw_rate)


def test_start_picks_the_open_end_and_then_keeps_it() -> None:
    at_the_far_wall = scan(far=0.7, near=-9.3)
    first, direction = hallway_command(at_the_far_wall)
    assert first.state == STATE_ALIGNING  # turn around: the open end is behind
    assert abs(first.heading_deg) == pytest.approx(180, abs=2)
    forward = np.array([1.0, 0.0])
    again, _ = hallway_command(at_the_far_wall, previous=forward, recent_front=1.2)
    assert again.state == STATE_ARRIVED
    assert again.front_m == pytest.approx(0.7, abs=0.02)


def test_arrival_needs_a_slow_approach() -> None:
    forward = np.array([1.0, 0.0])
    sudden, _ = hallway_command(scan(far=0.7, near=-9.3), previous=forward, recent_front=6.0)
    assert sudden.state == STATE_WALKING
    assert sudden.vx == 0.0
    unknown, _ = hallway_command(scan(far=0.7, near=-9.3), previous=forward, recent_front=None)
    assert unknown.state == STATE_WALKING


def test_slows_before_the_end_wall() -> None:
    decision, _ = hallway_command(scan(far=2.0, near=-8.0), previous=np.array([1.0, 0.0]))
    assert decision.state == STATE_WALKING
    assert PARAMS.min_speed <= decision.vx < PARAMS.cruise_speed


def test_side_wall_is_never_the_wall_ahead() -> None:
    decision, _ = hallway_command(scan(y=-0.67, yaw_deg=25))
    # The hugged wall crosses the strip ahead of a robot turned towards it; only
    # the far wall (sparse at 9 m, possibly unseen) may count as clearance.
    assert decision.front_m > 8.0
    assert decision.right_m == pytest.approx(0.33, abs=0.03)
    assert decision.state == STATE_WALKING


def test_no_walls_is_blind() -> None:
    decision, direction = hallway_command(scan(far=40, near=-40, width=40))
    assert decision.state == STATE_BLIND
    assert decision.vx == 0.0 and decision.wz == 0.0
    assert direction is None
    assert hallway_command(None)[0].state == STATE_BLIND


def test_decision_json_has_no_infinities() -> None:
    decision, _ = hallway_command(scan(far=40, near=-40, width=40))
    data = decision.to_json()
    assert data["front_m"] is None and data["left_m"] is None
    assert data["state"] == STATE_BLIND


class Loco:
    def __init__(self, mode: str = "walk") -> None:
        self.mode = mode
        self.paused = False
        self.moves: list[tuple[float, float, float]] = []

    def move(self, vx: float, vy: float, wz: float) -> None:
        self.moves.append((vx, vy, wz))


def _controller(loco: Loco):
    published: list[dict] = []
    clock = [0.0]
    controller = NavController(loco, published.append, clock=lambda: clock[0])
    return controller, published, clock


def _message(ranges: list[float]) -> LaserScan:
    return LaserScan(0, -math.pi, STEP, 10.0, ranges, [0.0, 0.0, 1.3], [1.0, 0.0, 0.0, 0.0])


def test_controller_refuses_to_start_unless_walking() -> None:
    controller, published, _ = _controller(Loco(mode="stand"))
    ok, detail = controller.start()
    assert not ok and "loco/start" in detail
    assert controller.tick() is None
    assert published == []


def test_controller_walks_the_hallway_and_arrives() -> None:
    loco = Loco()
    controller, published, clock = _controller(loco)
    assert controller.start()[0]
    x, y, yaw = 0.0, -0.3, math.radians(-50)
    for _ in range(1500):
        controller.on_scan(_message(synthetic(x=x, y=y, yaw_deg=math.degrees(yaw))))
        decision = controller.tick()
        assert decision is not None
        if decision.state == STATE_ARRIVED:
            break
        vx, _, wz = loco.moves[-1]
        x += vx * math.cos(yaw) * 0.1
        y += vx * math.sin(yaw) * 0.1
        yaw += (wz - 0.14) * 0.1  # the gait drifts in yaw; the controller must hold heading
        clock[0] += 0.1
    assert decision.state == STATE_ARRIVED
    assert not controller.active
    assert x > 7.0 and abs(y) < 0.4
    assert loco.moves[-1] == (0.0, 0.0, 0.0)
    assert [p["state"] for p in published][:2] == [STATE_WALKING, STATE_ALIGNING]
    assert published[-1]["state"] == STATE_ARRIVED


def test_controller_blind_without_a_recent_scan() -> None:
    loco = Loco()
    controller, published, clock = _controller(loco)
    controller.start()
    controller.on_scan(_message(synthetic()))
    clock[0] += 5.0
    decision = controller.tick()
    assert decision is not None and decision.state == STATE_BLIND
    assert loco.moves[-1] == (0.0, 0.0, 0.0)


def test_stop_zeroes_velocity_and_reports() -> None:
    loco = Loco()
    controller, published, _ = _controller(loco)
    assert controller.stop() == (True, "navigation was not running")
    controller.start()
    controller.on_scan(_message(synthetic()))
    controller.tick()
    assert loco.moves[-1][0] > 0
    assert controller.stop("teleop") == (True, "navigation stopped")
    assert loco.moves[-1] == (0.0, 0.0, 0.0)
    assert controller.state == STATE_STOPPED
    assert published[-1]["state"] == STATE_STOPPED


def test_mode_change_mid_walk_stops_the_controller() -> None:
    loco = Loco()
    controller, published, _ = _controller(loco)
    controller.start()
    controller.on_scan(_message(synthetic()))
    loco.mode = "damp"
    decision = controller.tick()
    assert decision is not None and decision.state == STATE_STOPPED
    assert not controller.active
