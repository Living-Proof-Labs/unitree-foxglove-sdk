"""A tiny event MCAP for each health episode.

The file carries a time window and nothing else. Foxlet uploads everything under
``events/`` because of the directory. A separate trigger reads the metadata and
creates the Foxglove event. This process never calls the Foxglove API.

An incident is one file per fall: 30 s before health entered ``incident``
through 10 s after. A degraded episode is one file for a run of ``degraded``
samples, including gaps shorter than a few seconds. It is written once that
quiet stretch has passed, so a bump that recovers is one report and a stumble
that becomes a fall is only the incident.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import foxglove

from unitree_foxglove.health import DEGRADED, INCIDENT, HealthState

PRE_ROLL_S = 30.0
POST_ROLL_S = 10.0
DEGRADED_PRE_ROLL_S = 10.0
DEGRADED_QUIET_S = 3.0
"""Seconds without a ``degraded`` sample that close one episode."""
EVENTS_DIR = "events"
METADATA_NAME = "event"
PRODUCER = "unitree-foxglove"

_NS_PER_S = 1_000_000_000


def iso_from_ns(nanoseconds: int) -> str:
    """UTC ISO-8601 with milliseconds and a ``Z``, as the Foxglove API accepts."""
    moment = datetime.fromtimestamp(nanoseconds / _NS_PER_S, tz=timezone.utc)
    return moment.strftime("%Y-%m-%dT%H:%M:%S.") + f"{moment.microsecond // 1000:03d}Z"


def ns_from_iso(text: str) -> int:
    """Inverse of :func:`iso_from_ns`."""
    moment = datetime.strptime(text, "%Y-%m-%dT%H:%M:%S.%fZ").replace(tzinfo=timezone.utc)
    return int(moment.timestamp() * _NS_PER_S)


def event_files_for(recording: Path) -> list[Path]:
    """Event MCAPs written for this telemetry file, oldest first."""
    directory = recording.parent / EVENTS_DIR
    if not directory.is_dir():
        return []
    return sorted(directory.glob(f"{recording.stem}-incident-*.mcap"))


def read_event_metadata(path: Path) -> dict[str, str]:
    """The ``event`` metadata record. Raises ``KeyError`` when it is absent."""
    from mcap.reader import make_reader

    with path.open("rb") as handle:
        records = [
            record for record in make_reader(handle).iter_metadata() if record.name == METADATA_NAME
        ]
    if len(records) != 1:
        raise KeyError(f"{path.name} has {len(records)} {METADATA_NAME!r} metadata records")
    return dict(records[0].metadata)


def event_mcap_path(recording: Path, at_ns: int, kind: str = INCIDENT) -> Path:
    """``<recording dir>/events/<run>-<kind>-<UTC>.mcap``.

    ``kind`` is ``incident`` or ``degraded``, so the two are distinct in a
    directory listing. Foxlet imports both: its pattern is the directory.
    """
    stamp = datetime.fromtimestamp(at_ns / _NS_PER_S, tz=timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    directory = recording.parent / EVENTS_DIR
    candidate = directory / f"{recording.stem}-{kind}-{stamp}.mcap"
    index = 2
    while candidate.exists():
        candidate = directory / f"{recording.stem}-{kind}-{stamp}-{index}.mcap"
        index += 1
    return candidate


@dataclass
class _Pending:
    kind: str
    since_ns: int
    start_ns: int
    end_ns: int
    reason: str
    source_name: str
    written: bool = False
    left_ns: int | None = None
    """When health last left ``degraded`` during this episode. Incidents ignore it."""


class IncidentRecorder:
    """Writes one metadata MCAP per incident and per recovered degraded episode.

    An incident window is ``since - 30 s`` (not before the recording started)
    through ``since + 10 s``. It does not wait for the robot to recover. A
    degraded episode opens on the first ``degraded`` sample, stays open across
    later ``degraded`` samples, and is written after
    :data:`DEGRADED_QUIET_S` with no ``degraded`` sample. Its window starts
    :data:`DEGRADED_PRE_ROLL_S` before the first sample and ends when that quiet
    stretch elapses. A ``degraded`` run that becomes ``incident`` without
    recovering is dropped: the fall is the incident file.

    ``finish`` writes an open window early, with ``end`` at the moment the
    recording stops. ``recording`` is the telemetry file new episodes name in
    ``source_recording``; the bridge updates it when that file rolls.
    """

    def __init__(
        self,
        recording: Path,
        start_ns: int,
        pre_roll_s: float = PRE_ROLL_S,
        post_roll_s: float = POST_ROLL_S,
        degraded_pre_roll_s: float = DEGRADED_PRE_ROLL_S,
        degraded_quiet_s: float = DEGRADED_QUIET_S,
    ) -> None:
        self.recording = recording
        self._recording_start_ns = start_ns
        self._pre_roll_ns = int(pre_roll_s * _NS_PER_S)
        self._post_roll_ns = int(post_roll_s * _NS_PER_S)
        self._degraded_pre_ns = int(degraded_pre_roll_s * _NS_PER_S)
        self._degraded_quiet_ns = int(degraded_quiet_s * _NS_PER_S)
        self._incident: _Pending | None = None
        self._degraded: _Pending | None = None
        self.written: list[Path] = []

    def update(self, health: HealthState | None, now_ns: int) -> None:
        state = None if health is None else health.state
        if health is not None and state == INCIDENT:
            self._on_incident(health, now_ns)
        elif health is not None and state == DEGRADED:
            self._on_degraded(health, now_ns)
        elif self._degraded is not None and not self._degraded.written:
            if self._degraded.left_ns is None:
                self._degraded.left_ns = now_ns
        self._write_incident_if_due(now_ns)
        self._write_degraded_if_quiet(now_ns)

    def finish(self, now_ns: int) -> None:
        """Write an armed window now if its close time has not arrived."""
        self._close_early(self._incident, now_ns)
        self._close_early(self._degraded, now_ns)

    def _on_incident(self, health: HealthState, now_ns: int) -> None:
        degraded = self._degraded
        if degraded is not None and not degraded.written:
            if degraded.left_ns is None:
                self._degraded = None
            else:
                self._close_early(degraded, now_ns)
        if self._incident is None or self._incident.since_ns != health.since_ns:
            self._close_early(self._incident, now_ns)
            start_ns = max(self._recording_start_ns, health.since_ns - self._pre_roll_ns)
            self._incident = _Pending(
                kind=INCIDENT,
                since_ns=health.since_ns,
                start_ns=start_ns,
                end_ns=health.since_ns + self._post_roll_ns,
                reason=health.reason,
                source_name=self.recording.name,
            )

    def _on_degraded(self, health: HealthState, now_ns: int) -> None:
        episode = self._degraded
        if episode is None or episode.written:
            start_ns = max(self._recording_start_ns, health.since_ns - self._degraded_pre_ns)
            self._degraded = _Pending(
                kind=DEGRADED,
                since_ns=health.since_ns,
                start_ns=start_ns,
                end_ns=now_ns + self._degraded_quiet_ns,
                reason=health.reason,
                source_name=self.recording.name,
            )
            return
        episode.reason = health.reason
        episode.left_ns = None
        episode.end_ns = now_ns + self._degraded_quiet_ns

    def _close_early(self, pending: _Pending | None, now_ns: int) -> None:
        if pending is None or pending.written:
            return
        pending.end_ns = now_ns
        self._write(pending)

    def _write_incident_if_due(self, now_ns: int) -> None:
        pending = self._incident
        if pending is None or pending.written or now_ns < pending.end_ns:
            return
        self._write(pending)

    def _write_degraded_if_quiet(self, now_ns: int) -> None:
        pending = self._degraded
        if pending is None or pending.written or pending.left_ns is None:
            return
        if now_ns < pending.left_ns + self._degraded_quiet_ns:
            return
        pending.end_ns = pending.left_ns + self._degraded_quiet_ns
        self._write(pending)

    def _write(self, pending: _Pending) -> None:
        path = event_mcap_path(
            self.recording.with_name(pending.source_name), pending.since_ns, pending.kind
        )
        write_event_mcap(
            path,
            {
                "kind": pending.kind,
                "start": iso_from_ns(pending.start_ns),
                "end": iso_from_ns(pending.end_ns),
                "at": iso_from_ns(pending.since_ns),
                "reason": pending.reason,
                "source_recording": pending.source_name,
                "producer": PRODUCER,
            },
        )
        pending.written = True
        self.written.append(path)
        print(f"{pending.kind} event {path.name}", flush=True)


def write_event_mcap(path: Path, metadata: dict[str, str]) -> None:
    """Metadata record plus two messages that mark the window.

    Foxglove dates a recording from its message times. A file with only a
    metadata record is dated 1970-01-01, so it cannot overlap the telemetry.
    ``/event`` is logged at ``start`` and at ``end``. A private context keeps
    the file off the live server and out of the telemetry recording.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    partial = path.with_name(path.name + ".active")
    context = foxglove.Context()
    with foxglove.open_mcap(partial, context=context) as writer:
        writer.write_metadata(METADATA_NAME, metadata)
        channel = foxglove.Channel("/event", message_encoding="json", context=context)
        start_ns = ns_from_iso(metadata["start"])
        end_ns = ns_from_iso(metadata["end"])
        channel.log(dict(metadata), log_time=start_ns)
        if end_ns != start_ns:
            channel.log(dict(metadata), log_time=end_ns)
    partial.replace(path)
