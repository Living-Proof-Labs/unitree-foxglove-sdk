"""Incident event files: a fixed window and a metadata-only MCAP."""

from pathlib import Path

from mcap.reader import make_reader

from unitree_foxglove.health import DEGRADED, INCIDENT, NOMINAL, HealthState
from unitree_foxglove.incidents import (
    DEGRADED_QUIET_S,
    METADATA_NAME,
    PRODUCER,
    IncidentRecorder,
    event_mcap_path,
    iso_from_ns,
)

S = 1_000_000_000
T0 = 1_790_640_000 * S  # 2026-09-29T00:00:00Z


def _state(state: str, since_s: float, reason: str = "pelvis 0.21 m below 0.45 m") -> HealthState:
    return HealthState(state, reason, T0 + int(since_s * S))


def _recorder(tmp_path: Path) -> IncidentRecorder:
    return IncidentRecorder(tmp_path / "g1-run.mcap", T0)


def _metadata(path: Path) -> dict[str, str]:
    with path.open("rb") as handle:
        records = [record for record in make_reader(handle).iter_metadata()]
    assert [record.name for record in records] == [METADATA_NAME]
    return dict(records[0].metadata)


def test_iso_from_ns() -> None:
    assert iso_from_ns(T0 + 123_456_789) == "2026-09-29T00:00:00.123Z"


def test_event_path_uses_the_incident_time(tmp_path: Path) -> None:
    recording = tmp_path / "g1-20260929T000000Z.mcap"
    path = event_mcap_path(recording, T0 + 37 * S)
    assert path == tmp_path / "events" / "g1-20260929T000000Z-incident-20260929T000037Z.mcap"


def test_window_is_written_once_thirty_before_and_ten_after(tmp_path: Path) -> None:
    recorder = _recorder(tmp_path)
    recorder.update(_state(NOMINAL, 0), T0 + 20 * S)
    recorder.update(_state(INCIDENT, 40), T0 + 40 * S)
    recorder.update(_state(INCIDENT, 40), T0 + 49 * S)
    assert recorder.written == []

    recorder.update(_state(NOMINAL, 45), T0 + 50 * S)
    (path,) = recorder.written
    assert _metadata(path) == {
        "kind": INCIDENT,
        "start": iso_from_ns(T0 + 10 * S),
        "end": iso_from_ns(T0 + 50 * S),
        "at": iso_from_ns(T0 + 40 * S),
        "reason": "pelvis 0.21 m below 0.45 m",
        "source_recording": "g1-run.mcap",
        "producer": PRODUCER,
    }
    with path.open("rb") as handle:
        times = [
            message.log_time for _schema, _channel, message in make_reader(handle).iter_messages()
        ]
    assert times == [T0 + 10 * S, T0 + 50 * S]

    recorder.update(_state(NOMINAL, 45), T0 + 80 * S)
    assert len(recorder.written) == 1


def test_a_later_incident_writes_another_file(tmp_path: Path) -> None:
    recorder = _recorder(tmp_path)
    recorder.update(_state(INCIDENT, 40), T0 + 50 * S)
    recorder.update(_state(NOMINAL, 55), T0 + 55 * S)
    recorder.update(_state(INCIDENT, 70, "tilt 80 deg past 60 deg"), T0 + 80 * S)
    assert len(recorder.written) == 2
    assert _metadata(recorder.written[1])["at"] == iso_from_ns(T0 + 70 * S)
    assert _metadata(recorder.written[1])["reason"] == "tilt 80 deg past 60 deg"


def test_pre_roll_is_clamped_and_finish_closes_early(tmp_path: Path) -> None:
    recorder = _recorder(tmp_path)
    recorder.update(_state(INCIDENT, 3), T0 + 4 * S)
    recorder.finish(T0 + 8 * S)
    meta = _metadata(recorder.written[0])
    assert meta["start"] == iso_from_ns(T0)
    assert meta["end"] == iso_from_ns(T0 + 8 * S)
    assert meta["at"] == iso_from_ns(T0 + 3 * S)


def test_no_incident_writes_no_file(tmp_path: Path) -> None:
    recorder = _recorder(tmp_path)
    recorder.update(_state(NOMINAL, 0), T0 + S)
    recorder.finish(T0 + 2 * S)
    assert recorder.written == []
    assert not (tmp_path / "events").exists()


def test_a_recovered_bump_is_one_degraded_file(tmp_path: Path) -> None:
    recorder = _recorder(tmp_path)
    recorder.update(_state(DEGRADED, 20, "tilt 30 deg past 25 deg"), T0 + 20 * S)
    recorder.update(_state(DEGRADED, 20, "tilt 40 deg past 25 deg"), T0 + 22 * S)
    recorder.update(_state(NOMINAL, 23), T0 + 23 * S)
    recorder.update(_state(DEGRADED, 24, "pelvis 0.60 m below 0.65 m"), T0 + 24 * S)
    recorder.update(_state(NOMINAL, 25), T0 + 25 * S)
    assert recorder.written == []
    recorder.update(_state(NOMINAL, 25), T0 + int((25 + DEGRADED_QUIET_S) * S))
    (path,) = recorder.written
    assert path.name.startswith("g1-run-degraded-")
    assert _metadata(path) == {
        "kind": DEGRADED,
        "start": iso_from_ns(T0 + 10 * S),
        "end": iso_from_ns(T0 + int((25 + DEGRADED_QUIET_S) * S)),
        "at": iso_from_ns(T0 + 20 * S),
        "reason": "pelvis 0.60 m below 0.65 m",
        "source_recording": "g1-run.mcap",
        "producer": PRODUCER,
    }


def test_a_quiet_gap_splits_degraded_episodes(tmp_path: Path) -> None:
    recorder = _recorder(tmp_path)
    recorder.update(_state(DEGRADED, 20, "first"), T0 + 20 * S)
    recorder.update(_state(NOMINAL, 21), T0 + 21 * S)
    recorder.update(_state(NOMINAL, 21), T0 + int((21 + DEGRADED_QUIET_S) * S))
    recorder.update(_state(DEGRADED, 40, "second"), T0 + 40 * S)
    recorder.update(_state(NOMINAL, 41), T0 + 41 * S)
    recorder.update(_state(NOMINAL, 41), T0 + int((41 + DEGRADED_QUIET_S) * S))
    assert [path.name.split("-")[2] for path in recorder.written] == ["degraded", "degraded"]
    assert _metadata(recorder.written[0])["reason"] == "first"
    assert _metadata(recorder.written[1])["reason"] == "second"
    assert _metadata(recorder.written[1])["at"] == iso_from_ns(T0 + 40 * S)


def test_a_fall_does_not_also_write_the_degraded_hold(tmp_path: Path) -> None:
    recorder = _recorder(tmp_path)
    recorder.update(_state(DEGRADED, 20, "pelvis 0.50 m below 0.65 m"), T0 + 20 * S)
    recorder.update(_state(INCIDENT, 21), T0 + 21 * S)
    assert recorder.written == []
    recorder.update(_state(INCIDENT, 21), T0 + 31 * S)
    (path,) = recorder.written
    assert _metadata(path)["kind"] == INCIDENT


def test_duplicate_stamp_gets_a_suffix(tmp_path: Path) -> None:
    recording = tmp_path / "g1-run.mcap"
    first = event_mcap_path(recording, T0)
    first.parent.mkdir()
    first.write_bytes(b"")
    assert event_mcap_path(recording, T0).name == "g1-run-incident-20260929T000000Z-2.mcap"
