"""Walk down an unknown hallway to the wall that closes it.

The behaviour needs no map and no localisation. Every tick it reads one planar
scan and finds the corridor in it: the dominant wall direction across the whole
scan is the axis, the returns that lie along that axis are the side walls, and
the nearest return inside the body's strip along the axis is the wall ahead.
The robot turns to face along the axis, steers to stay midway between the side
walls, slows as the wall ahead approaches, and stops in front of it.

Which way along the axis is decided once, when navigation starts: the more open
end. After that the controller keeps following the same end while the robot
turns, so a policy that lands the robot facing a side wall, or a walking gait
that drifts in yaw, still ends at the far wall. The policy is the same in a
10 m hallway and a 20 m one, and the same on a real G1 whose head lidar is
reduced to a planar scan.

:func:`hallway_command` is pure: scan in, velocity command and diagnostics out.
:class:`NavController` runs it at a fixed rate against a locomotion backend.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import asdict, dataclass, replace

import numpy as np

TOPIC_NAV_STATE = "/nav/state"

STATE_IDLE = "idle"
"""Not navigating; Teleop drives."""
STATE_ALIGNING = "aligning"
"""Turning in place to face along the corridor."""
STATE_WALKING = "walking"
STATE_ARRIVED = "arrived"
"""Stopped in front of the wall that ends the hallway."""
STATE_BLIND = "blind"
"""No walls in range, or no recent scan; holding still."""
STATE_STOPPED = "stopped"
"""Cancelled by the operator (service, Teleop, or a mode change)."""

RATE_HZ = 10.0
SCAN_STALE_S = 0.5


@dataclass(frozen=True)
class NavParams:
    """Tuning for :func:`hallway_command`. Units are metres, radians, seconds."""

    cruise_speed: float = 0.3
    """Commanded, not achieved: the pretrained gait covers ground faster than asked."""
    min_speed: float = 0.1
    stop_distance_m: float = 1.0
    """Clearance ahead at which the robot declares it has arrived."""
    slow_distance_m: float = 3.5
    """Clearance ahead below which the speed ramps down towards ``min_speed``."""
    center_gain: float = 0.8
    """Heading bias (radians) per unit of normalised offset (-1 at one wall, +1 at the other)."""
    heading_gain: float = 1.0
    """Yaw rate per radian of heading error, corridor axis plus centring bias."""
    max_yaw_rate: float = 0.5
    walk_yaw_rate: float = 0.4
    """Yaw rate cap while walking; the gait tolerates less turning than a turn in place."""
    yaw_smoothing: float = 0.3
    """Weight of the newest yaw command in the exponential average sent to the gait."""
    approach_window_s: float = 1.0
    approach_max_m: float = 0.8
    """Arrival counts only if the wall ahead was within ``stop + this`` a window ago.

    A fall sweeps the lidar towards the floor and the wall in a fraction of a
    second; a walk closes the last metre slowly.
    """
    align_rad: float = math.radians(30)
    """Heading error above which the robot turns in place instead of walking."""
    corridor_half_m: float = 0.5
    """Half width of the strip ahead that must be clear; side walls fall outside it."""
    segment_max_m: float = 0.8
    """Adjacent returns farther apart than this are not on the same wall."""
    axis_tolerance_rad: float = math.radians(15)
    """Returns whose local direction is this close to the axis count as side wall."""
    min_wall_m: float = 1.0
    """Total length of wall-aligned segments needed before the axis is trusted."""
    side_min_m: float = 0.05
    """Offsets closer than this to the axis line through the robot are not a wall beside us."""
    range_min_m: float = 0.5
    """Returns closer than this are the robot's own arms swinging through the fan."""
    wall_band_m: float = 0.15
    """Returns this close to a side wall's line belong to it, however sparse."""
    wall_band_slope: float = 0.03
    """Extra band per metre along the axis: about 2 degrees of axis error."""
    center_max_rad: float = math.radians(30)
    """Largest heading bias the centre-line offset may add."""


PARAMS = NavParams()


@dataclass(frozen=True)
class Scan:
    """Bearings (radians, counter-clockwise from +x) and ranges (metres, inf = no return)."""

    angles: np.ndarray
    ranges: np.ndarray

    @classmethod
    def from_dds(cls, message: object, range_min: float = 0.0) -> Scan:
        """Build from a ``LaserScan`` DDS sample; returns under ``range_min`` become inf.

        The head lidar sees the robot's own hands when the gait swings the
        arms up. A lidar driver on a real robot blanks that near field the
        same way.
        """
        ranges = np.asarray(message.ranges, dtype=float).copy()  # type: ignore[attr-defined]
        ranges[ranges < range_min] = np.inf
        angles = float(message.angle_min) + float(message.angle_increment) * np.arange(  # type: ignore[attr-defined]
            len(ranges)
        )
        return cls(angles=angles, ranges=ranges)

    def points(self) -> np.ndarray:
        """Cartesian returns in bearing order, shape (n, 2); ``nan`` rows where none."""
        finite = np.isfinite(self.ranges)
        pts = np.full((len(self.ranges), 2), np.nan)
        pts[finite, 0] = self.ranges[finite] * np.cos(self.angles[finite])
        pts[finite, 1] = self.ranges[finite] * np.sin(self.angles[finite])
        return pts


@dataclass(frozen=True)
class Corridor:
    """The hallway as seen in the robot frame."""

    direction: np.ndarray
    """Unit vector along the walls, pointing the way to go."""
    front_m: float
    left_m: float
    right_m: float

    @property
    def heading_rad(self) -> float:
        """Turn needed to face ``direction``; positive is left (counter-clockwise)."""
        return math.atan2(float(self.direction[1]), float(self.direction[0]))


@dataclass(frozen=True)
class Decision:
    """What the controller decided this tick and what it saw."""

    state: str
    vx: float
    wz: float
    front_m: float
    left_m: float
    right_m: float
    heading_deg: float

    def to_json(self) -> dict:
        data = asdict(self)
        for key in ("front_m", "left_m", "right_m"):
            if not math.isfinite(data[key]):
                data[key] = None
        return data


def _segments(points: np.ndarray, max_len: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Vectors between consecutive returns on the same surface; returns (vectors, i, j)."""
    n = len(points)
    if n < 2:
        empty = np.zeros((0, 2))
        return empty, np.zeros(0, dtype=int), np.zeros(0, dtype=int)
    i = np.arange(n)
    j = (i + 1) % n  # the scan wraps around
    vectors = points[j] - points[i]
    lengths = np.hypot(vectors[:, 0], vectors[:, 1])
    keep = np.isfinite(lengths) & (lengths > 1e-6) & (lengths <= max_len)
    return vectors[keep], i[keep], j[keep]


def _axis_angle(
    vectors: np.ndarray, prior: float | None = None, prior_window: float = math.radians(45)
) -> tuple[float, float] | None:
    """Dominant undirected direction of the segments, and the wall length behind it.

    Angles are doubled so a wall and its opposite side agree, then the
    length-weighted vote picks the axis and a weighted mean sharpens it. With a
    ``prior`` axis only directions within ``prior_window`` of it compete, so the
    wall that closes the hallway cannot out-vote the side walls at the end.
    """
    lengths = np.hypot(vectors[:, 0], vectors[:, 1])
    if lengths.sum() <= 0:
        return None
    doubled = 2 * np.arctan2(vectors[:, 1], vectors[:, 0])
    bins = 36
    index = np.floor(((doubled + np.pi) / (2 * np.pi)) * bins).astype(int) % bins
    votes = np.bincount(index, weights=lengths, minlength=bins)
    smoothed = votes + np.roll(votes, 1) + np.roll(votes, -1)
    if prior is not None:
        centers = -np.pi + (np.arange(bins) + 0.5) * (2 * np.pi / bins)
        away = np.abs(np.angle(np.exp(1j * (centers - 2 * prior)))) / 2
        smoothed = np.where(away <= prior_window, smoothed, 0.0)
        if not smoothed.any():
            return None
    best = int(np.argmax(smoothed))
    center = -np.pi + (best + 0.5) * (2 * np.pi / bins)
    near = np.abs(np.angle(np.exp(1j * (doubled - center)))) <= 1.5 * (2 * np.pi / bins)
    if not near.any():
        return None
    mean = np.sum(lengths[near] * np.exp(1j * doubled[near]))
    return float(np.angle(mean) / 2), float(lengths[near].sum())


def find_corridor(
    scan: Scan, params: NavParams = PARAMS, previous: np.ndarray | None = None
) -> Corridor | None:
    """Fit the corridor to one scan.

    ``previous`` is the direction chosen last tick, in the robot frame; the
    end of the axis nearest to it is kept so the robot does not change its
    mind while turning. Without it the more open end is chosen.
    """
    points = scan.points()
    vectors, i, j = _segments(points, params.segment_max_m)
    prior = None if previous is None else math.atan2(float(previous[1]), float(previous[0]))
    axis = _axis_angle(vectors, prior)
    if axis is None or axis[1] < params.min_wall_m:
        return None
    angle, _ = axis
    u = np.array([math.cos(angle), math.sin(angle)])

    # Returns that lie along the axis are side wall. Everything else that is in
    # the strip ahead is what we must not walk into: the side wall itself never
    # counts, even when the robot hugs it.
    seg_angles = np.arctan2(vectors[:, 1], vectors[:, 0])
    diff = np.angle(np.exp(2j * (seg_angles - angle))) / 2
    aligned = np.abs(diff) <= params.axis_tolerance_rad
    is_wall = np.zeros(len(points), dtype=bool)
    is_wall[np.concatenate((i[aligned], j[aligned]))] = True
    finite = np.isfinite(points[:, 0])

    if previous is not None:
        direction = u if float(np.dot(u, previous)) >= 0 else -u
    else:
        direction = u  # settled below, once the walls are known
    normal = np.array([-direction[1], direction[0]])  # left of the way to go
    offsets = points @ normal
    wall_offsets = offsets[is_wall]
    left = wall_offsets[wall_offsets >= params.side_min_m]
    right = -wall_offsets[wall_offsets <= -params.side_min_m]
    left_m = float(np.median(left)) if len(left) else math.inf
    right_m = float(np.median(right)) if len(right) else math.inf
    # A wall seen at a grazing angle thins out beyond the segment limit; its
    # far returns still sit at the wall's offset, so they are wall too.
    along = np.abs(points @ direction)
    band = params.wall_band_m + params.wall_band_slope * along  # axis error grows with range
    for wall in (left_m, -right_m):
        if math.isfinite(wall):
            is_wall |= finite & (np.abs(offsets - wall) <= band)
    obstacles = points[finite & ~is_wall]

    if previous is None:
        ahead, behind = (_clearance(obstacles, d, params) for d in (u, -u))
        if ahead < behind or (ahead == behind and u[0] < 0):
            direction = -u
            left_m, right_m = right_m, left_m
    return Corridor(
        direction=direction,
        front_m=_clearance(obstacles, direction, params),
        left_m=left_m,
        right_m=right_m,
    )


def _clearance(points: np.ndarray, direction: np.ndarray, params: NavParams) -> float:
    """Nearest return along ``direction`` inside the strip the body has to pass through."""
    if len(points) == 0:
        return math.inf
    along = points @ direction
    across = points @ np.array([-direction[1], direction[0]])
    inside = (along > 0) & (np.abs(across) <= params.corridor_half_m)
    return float(along[inside].min()) if inside.any() else math.inf


def hallway_command(
    scan: Scan | None,
    params: NavParams = PARAMS,
    previous: np.ndarray | None = None,
    recent_front: float | None = None,
) -> tuple[Decision, np.ndarray | None]:
    """One step of the policy. Returns the decision and the direction to remember.

    The heading target is the corridor axis biased towards the centre line, so
    a robot hugging a wall turns away from it before it walks. It turns in
    place while more than ``align_rad`` off that target, otherwise walks, and
    stops when the strip ahead is shorter than ``stop_distance_m``, provided
    ``recent_front`` (the clearance one window ago) shows a slow approach.
    """
    blind = Decision(STATE_BLIND, 0.0, 0.0, math.inf, math.inf, math.inf, 0.0)
    if scan is None:
        return blind, None
    corridor = find_corridor(scan, params, previous)
    if corridor is None or (math.isinf(corridor.left_m) and math.isinf(corridor.right_m)):
        return blind, None
    heading = corridor.heading_rad
    offset = 0.0
    if math.isfinite(corridor.left_m) and math.isfinite(corridor.right_m):
        offset = (corridor.left_m - corridor.right_m) / (corridor.left_m + corridor.right_m)
    bias = max(-params.center_max_rad, min(params.center_max_rad, params.center_gain * offset))
    turn = heading + bias
    wz = max(-params.max_yaw_rate, min(params.max_yaw_rate, params.heading_gain * turn))
    seen = (corridor.front_m, corridor.left_m, corridor.right_m, math.degrees(heading))

    if abs(turn) > params.align_rad:
        return Decision(STATE_ALIGNING, 0.0, wz, *seen), corridor.direction
    if corridor.front_m <= params.stop_distance_m:
        approached = (
            recent_front is not None
            and recent_front <= params.stop_distance_m + params.approach_max_m
        )
        if approached:
            return Decision(STATE_ARRIVED, 0.0, 0.0, *seen), corridor.direction
        # Too sudden to be a wall we walked up to: hold and look again.
        return Decision(STATE_WALKING, 0.0, 0.0, *seen), corridor.direction
    ramp = (corridor.front_m - params.stop_distance_m) / (
        params.slow_distance_m - params.stop_distance_m
    )
    vx = params.cruise_speed * max(params.min_speed / params.cruise_speed, min(1.0, ramp))
    vx *= max(0.5, math.cos(turn))  # ease off while still turning
    wz = max(-params.walk_yaw_rate, min(params.walk_yaw_rate, wz))
    return Decision(STATE_WALKING, vx, wz, *seen), corridor.direction


class Mover:
    """The part of ``LocoBackend`` the controller needs; keeps ``nav`` free of ``bridge``."""

    mode: str

    def move(self, vx: float, vy: float, wz: float) -> None: ...

    @property
    def paused(self) -> bool: ...


class NavController:
    """Runs :func:`hallway_command` at :data:`RATE_HZ` while active.

    ``publish`` receives each decision as JSON for ``/nav/state``. Starting
    requires the walking policy to be running; the controller never changes
    the locomotion mode itself, it only sends velocities.
    """

    def __init__(
        self,
        loco: Mover,
        publish: Callable[[dict], None],
        params: NavParams = PARAMS,
        walk_mode: str = "walk",
        rate_hz: float = RATE_HZ,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._loco = loco
        self._publish = publish
        self._params = params
        self._walk_mode = walk_mode
        self._period = 1.0 / rate_hz
        self._clock = clock
        self._lock = threading.Lock()
        self._scan: Scan | None = None
        self._scan_at = -math.inf
        self._active = False
        self._direction: np.ndarray | None = None
        self._fronts: deque[tuple[float, float]] = deque()
        self._wz = 0.0
        self.state = STATE_IDLE
        self.last: Decision | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def active(self) -> bool:
        return self._active

    def announce(self) -> None:
        """Publish the idle state so the topic exists before the first start."""
        self._publish(self._idle_decision(self.state).to_json())

    def on_scan(self, message: object) -> None:
        scan = Scan.from_dds(message, self._params.range_min_m)
        with self._lock:
            self._scan = scan
            self._scan_at = self._clock()

    def start(self) -> tuple[bool, str]:
        """Begin walking. Returns ``(ok, detail)`` for the service response."""
        if self._loco.mode != self._walk_mode or self._loco.paused:
            return False, "start the walking policy first (loco/start), then nav/start"
        with self._lock:
            self._active = True
            self._direction = None
            self._fronts.clear()
            self._wz = 0.0
            self.state = STATE_WALKING
        self._publish(self._idle_decision(STATE_WALKING).to_json())
        return True, "walking to the end of the hallway"

    def stop(self, reason: str = "operator") -> tuple[bool, str]:
        """Hold position and hand control back to Teleop."""
        with self._lock:
            was_active = self._active
            self._active = False
            self.state = STATE_STOPPED if was_active else self.state
        if was_active:
            self._loco.move(0.0, 0.0, 0.0)
            self._publish(self._idle_decision(STATE_STOPPED).to_json())
            print(f"nav stopped: {reason}", flush=True)
        return True, "navigation stopped" if was_active else "navigation was not running"

    def tick(self) -> Decision | None:
        """One control step; ``None`` while inactive."""
        now = self._clock()
        with self._lock:
            if not self._active:
                return None
            scan = self._scan if now - self._scan_at <= SCAN_STALE_S else None
            previous = self._direction
            recent = self._recent_front(now)
        decision, direction = hallway_command(scan, self._params, previous, recent)
        if self._loco.mode != self._walk_mode:
            decision = Decision(STATE_STOPPED, 0.0, 0.0, math.inf, math.inf, math.inf, 0.0)
        # The gait sways the torso, and the lidar with it, so the raw heading
        # jitters by several degrees per step. Average before commanding.
        alpha = self._params.yaw_smoothing
        self._wz = (
            decision.wz
            if decision.vx == 0.0 and decision.wz == 0.0
            else (alpha * decision.wz + (1.0 - alpha) * self._wz)
        )
        decision = replace(decision, wz=self._wz)
        self._loco.move(decision.vx, 0.0, decision.wz)
        with self._lock:
            self._fronts.append((now, decision.front_m))
            self.last = decision
            self.state = decision.state
            self._direction = direction
            if decision.state in (STATE_ARRIVED, STATE_STOPPED):
                self._active = False
        self._publish(decision.to_json())
        if decision.state == STATE_ARRIVED:
            print(f"nav arrived: wall {decision.front_m:.2f} m ahead", flush=True)
        return decision

    def run_in_thread(self) -> None:
        self._thread = threading.Thread(target=self._run, name="nav", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        next_at = self._clock()
        while not self._stop.is_set():
            self.tick()
            next_at += self._period
            delay = next_at - self._clock()
            if delay > 0:
                self._stop.wait(delay)
            else:
                next_at = self._clock()

    def _recent_front(self, now: float) -> float | None:
        """The clearance ahead as it was one approach window ago, if known."""
        horizon = now - self._params.approach_window_s
        while len(self._fronts) > 1 and self._fronts[1][0] <= horizon:
            self._fronts.popleft()
        if self._fronts and self._fronts[0][0] <= horizon:
            return self._fronts[0][1]
        return None

    def _idle_decision(self, state: str) -> Decision:
        return Decision(state, 0.0, 0.0, math.inf, math.inf, math.inf, 0.0)
