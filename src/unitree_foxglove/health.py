"""Observe-only health state: is the robot fine.

The classifier reads pelvis height, IMU tilt, the requested loco mode, and the
rope state. It never commands anything. :func:`judge` is the instantaneous
verdict; :class:`HealthMonitor` adds the hold time that separates a stumble
from a fall, the offline timeout, and the publish cadence.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

TOPIC_HEALTH_STATE = "/health/state"

NOMINAL = "nominal"
DEGRADED = "degraded"
INCIDENT = "incident"
INACTIVE = "inactive"
OFFLINE = "offline"

_NS_PER_S = 1_000_000_000


@dataclass(frozen=True)
class Thresholds:
    fallen_height_m: float
    """Pelvis below this is a fall."""
    low_height_m: float
    """Pelvis below this, but above ``fallen_height_m``, is degraded."""
    fallen_tilt_deg: float = 60.0
    tilted_deg: float = 25.0
    incident_hold_s: float = 0.5
    """A fall condition must last this long before it is an incident."""
    offline_after_s: float = 1.0
    publish_period_s: float = 1.0


# Standing heights: G1 pelvis about 0.75 m, Go2 base about 0.3 m.
THRESHOLDS: dict[str, Thresholds] = {
    "g1": Thresholds(fallen_height_m=0.45, low_height_m=0.65),
    "go2": Thresholds(fallen_height_m=0.15, low_height_m=0.22),
}


@dataclass(frozen=True)
class Observation:
    time_ns: int
    height_m: float | None
    """``None`` until the first ``rt/sportmodestate`` sample."""
    tilt_deg: float
    mode: str
    """The ``/loco/mode`` label: ``damp``, ``stand``, ``walk``, or ``paused``."""
    band_enabled: bool


@dataclass(frozen=True)
class HealthState:
    state: str
    reason: str
    since_ns: int

    def to_json(self) -> dict:
        return {"state": self.state, "reason": self.reason, "since_ns": self.since_ns}


def tilt_from_quaternion(wxyz: Sequence[float]) -> float:
    """Degrees between the body z axis and world up, from a w, x, y, z quaternion."""
    _w, x, y, _z = (float(value) for value in wxyz)
    cosine = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def judge(observation: Observation, limits: Thresholds) -> tuple[str, str]:
    """State and reason for one observation, before any hold time."""
    if observation.band_enabled:
        return INACTIVE, "hanging on the elastic band"
    if observation.mode == "damp":
        return INACTIVE, "motors damped"
    height, tilt = observation.height_m, observation.tilt_deg
    if height is not None and height < limits.fallen_height_m:
        return INCIDENT, f"pelvis {height:.2f} m below {limits.fallen_height_m:.2f} m"
    if tilt > limits.fallen_tilt_deg:
        return INCIDENT, f"tilt {tilt:.0f} deg past {limits.fallen_tilt_deg:.0f} deg"
    if height is not None and height < limits.low_height_m:
        return DEGRADED, f"pelvis {height:.2f} m below {limits.low_height_m:.2f} m"
    if tilt > limits.tilted_deg:
        return DEGRADED, f"tilt {tilt:.0f} deg past {limits.tilted_deg:.0f} deg"
    return NOMINAL, f"upright, {observation.mode}"


class HealthMonitor:
    """Turns observations into a published health state.

    ``observe`` and ``tick`` return a :class:`HealthState` when it should be
    published: on every change of state and otherwise once per publish period.
    """

    def __init__(self, limits: Thresholds, start_ns: int) -> None:
        self._limits = limits
        self._last_observation_ns = start_ns
        self._fall_since_ns: int | None = None
        self._last_published_ns: int | None = None
        self.current: HealthState | None = None

    def observe(self, observation: Observation) -> HealthState | None:
        now = observation.time_ns
        self._last_observation_ns = now
        state, reason = judge(observation, self._limits)
        if state == INCIDENT:
            if self._fall_since_ns is None:
                self._fall_since_ns = now
            held_s = (now - self._fall_since_ns) / _NS_PER_S
            if held_s < self._limits.incident_hold_s and self._state() != INCIDENT:
                state = DEGRADED
            since = self._fall_since_ns
        else:
            self._fall_since_ns = None
            since = now
        return self._update(state, reason, since, now)

    def tick(self, now_ns: int) -> HealthState | None:
        """Call when no sample arrived; reports ``offline`` after the timeout."""
        silent_s = (now_ns - self._last_observation_ns) / _NS_PER_S
        if silent_s >= self._limits.offline_after_s:
            self._fall_since_ns = None
            return self._update(
                OFFLINE, f"no robot state for {silent_s:.1f} s", self._last_observation_ns, now_ns
            )
        return self._due(now_ns)

    def _state(self) -> str | None:
        return None if self.current is None else self.current.state

    def _update(self, state: str, reason: str, since_ns: int, now_ns: int) -> HealthState | None:
        if state != self._state():
            self.current = HealthState(state, reason, since_ns)
            self._last_published_ns = now_ns
            return self.current
        assert self.current is not None
        self.current = HealthState(state, reason, self.current.since_ns)
        return self._due(now_ns)

    def _due(self, now_ns: int) -> HealthState | None:
        if self.current is None or self._last_published_ns is None:
            return None
        if (now_ns - self._last_published_ns) / _NS_PER_S < self._limits.publish_period_s:
            return None
        self._last_published_ns = now_ns
        return self.current
