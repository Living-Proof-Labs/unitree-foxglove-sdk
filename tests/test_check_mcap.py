"""Round-trip a Foxglove MCAP and require the joint channel to show motion."""

import os
from pathlib import Path

import foxglove
import pytest
from foxglove.messages import (
    FrameTransform,
    FrameTransforms,
    JointState,
    JointStates,
    LaserScan,
    Pose,
    PoseInFrame,
    Timestamp,
    Vector3,
)

from unitree_foxglove.check_mcap import RecordingError, check_recording, resolve_mcap
from unitree_foxglove.convert import (
    TOPIC_BASE_POSE,
    TOPIC_BASE_VELOCITY,
    TOPIC_IMU_ACCEL,
    TOPIC_IMU_GYRO,
    TOPIC_JOINT_STATES,
    TOPIC_SCAN,
    TOPIC_TF,
)
from unitree_foxglove.health import TOPIC_HEALTH_STATE
from unitree_foxglove.incidents import PRODUCER, event_mcap_path, iso_from_ns, write_event_mcap
from unitree_foxglove.nav import TOPIC_NAV_STATE


def _write(
    path: Path,
    positions: list[float],
    base: list[tuple[float, float, float]] | None = None,
    health: list[str] | None = None,
    nav: list[str] | None = None,
    scans: bool = False,
) -> None:
    with foxglove.open_mcap(path):
        for index, position in enumerate(positions):
            log_time = index * 1_000_000
            if health:
                state = {"state": health[index], "reason": "", "since_ns": log_time}
                foxglove.log(TOPIC_HEALTH_STATE, state, log_time=log_time)
            if nav:
                foxglove.log(TOPIC_NAV_STATE, {"state": nav[index], "vx": 0.0}, log_time=log_time)
            if scans:
                foxglove.log(
                    TOPIC_SCAN,
                    LaserScan(timestamp=Timestamp(0, log_time), frame_id="world", ranges=[1.0]),
                    log_time=log_time,
                )
            foxglove.log(
                TOPIC_JOINT_STATES,
                JointStates(
                    timestamp=Timestamp(0, log_time),
                    joints=[JointState(name="FR_hip_joint", position=position)],
                ),
                log_time=log_time,
            )
            x, y, z = base[index] if base else (0.0, 0.0, 0.0)
            pose = PoseInFrame(
                timestamp=Timestamp(0, 0), pose=Pose(position=Vector3(x=x, y=y, z=z))
            )
            foxglove.log(TOPIC_BASE_POSE, pose, log_time=log_time)
            transform = FrameTransform(
                parent_frame_id="world",
                child_frame_id="base_link",
                translation=Vector3(x=x, y=y, z=z),
            )
            foxglove.log(TOPIC_TF, FrameTransforms(transforms=[transform]), log_time=log_time)
            foxglove.log(TOPIC_BASE_VELOCITY, Vector3(x=0.1), log_time=log_time)
            foxglove.log(TOPIC_IMU_GYRO, Vector3(x=0.01), log_time=log_time)
            foxglove.log(TOPIC_IMU_ACCEL, Vector3(z=9.81), log_time=log_time)


def test_check_recording_accepts_joint_travel(tmp_path: Path) -> None:
    path = tmp_path / "moved.mcap"
    _write(path, [0.0, 0.2])
    summary = check_recording(path)
    assert summary["joint_messages"] == 2
    assert summary["joint_travel"] == pytest.approx(0.2)


def test_check_recording_rejects_a_still_robot(tmp_path: Path) -> None:
    path = tmp_path / "still.mcap"
    _write(path, [0.1, 0.1])
    with pytest.raises(RecordingError, match="Joint travel"):
        check_recording(path)


def test_walk_check_needs_distance_without_a_fall(tmp_path: Path) -> None:
    walked = tmp_path / "walked.mcap"
    _write(walked, [0.0, 0.3, 0.2], [(0.0, 0.0, 0.79), (0.4, 0.0, 0.72), (0.9, 0.1, 0.74)])
    summary = check_recording(walked, min_forward_m=0.3)
    assert summary["forward"] == pytest.approx((0.9**2 + 0.1**2) ** 0.5)
    assert summary["min_z"] == pytest.approx(0.72)
    # Without the flag the base pose is not judged.
    assert "forward" not in check_recording(walked)

    fell = tmp_path / "fell.mcap"
    _write(fell, [0.0, 0.3, 0.2], [(0.0, 0.0, 0.79), (0.5, 0.0, 0.3), (0.9, 0.0, 0.2)])
    with pytest.raises(RecordingError, match="fell"):
        check_recording(fell, min_forward_m=0.3)

    stood = tmp_path / "stood.mcap"
    _write(stood, [0.0, 0.3, 0.2], [(0.0, 0.0, 0.79), (0.1, 0.0, 0.75), (0.1, 0.0, 0.75)])
    with pytest.raises(RecordingError, match="travelled"):
        check_recording(stood, min_forward_m=0.3)


def test_no_incident_check_reads_the_health_timeline(tmp_path: Path) -> None:
    fine = tmp_path / "fine.mcap"
    _write(fine, [0.0, 0.2, 0.1], health=["inactive", "nominal", "nominal"])
    assert check_recording(fine, no_incident=True)["health"] == "inactive -> nominal"

    fell = tmp_path / "fell.mcap"
    _write(fell, [0.0, 0.2, 0.1], health=["nominal", "degraded", "incident"])
    with pytest.raises(RecordingError, match="incident"):
        check_recording(fell, no_incident=True)

    silent = tmp_path / "silent.mcap"
    _write(silent, [0.0, 0.2])
    with pytest.raises(RecordingError, match="/health/state"):
        check_recording(silent, no_incident=True)


def test_nav_arrived_check_reads_scan_and_navigator_timeline(tmp_path: Path) -> None:
    arrived = tmp_path / "arrived.mcap"
    _write(arrived, [0.0, 0.2, 0.1], nav=["idle", "walking", "arrived"], scans=True)
    summary = check_recording(arrived, nav_arrived=True)
    assert summary["nav"] == "idle -> walking -> arrived"
    assert summary["scans"] == 3

    stopped = tmp_path / "stopped.mcap"
    _write(stopped, [0.0, 0.2, 0.1], nav=["idle", "walking", "stopped"], scans=True)
    with pytest.raises(RecordingError, match="'stopped'"):
        check_recording(stopped, nav_arrived=True)

    no_scan = tmp_path / "noscan.mcap"
    _write(no_scan, [0.0, 0.2], nav=["idle", "arrived"])
    with pytest.raises(RecordingError, match="/scan"):
        check_recording(no_scan, nav_arrived=True)

    assert "nav" not in check_recording(arrived)


def _event(recording: Path, at_ns: int, post_roll_s: float = 10.0) -> None:
    write_event_mcap(
        event_mcap_path(recording, at_ns),
        {
            "kind": "incident",
            "start": iso_from_ns(at_ns - int(30 * 1_000_000_000)),
            "end": iso_from_ns(at_ns + int(post_roll_s * 1_000_000_000)),
            "at": iso_from_ns(at_ns),
            "reason": "pelvis 0.21 m below 0.45 m",
            "source_recording": recording.name,
            "producer": PRODUCER,
        },
    )


def test_expect_incident_requires_health_and_one_event_file(tmp_path: Path) -> None:
    fell = tmp_path / "g1-run.mcap"
    at_ns = 1_790_640_040 * 1_000_000_000
    _write(fell, [0.0, 0.2, 0.1], health=["nominal", "degraded", "incident"])
    with pytest.raises(RecordingError, match="event MCAP"):
        check_recording(fell, expect_incident=True)

    _event(fell, at_ns)
    summary = check_recording(fell, expect_incident=True)
    assert summary["health"] == "nominal -> degraded -> incident"
    assert summary["event"].startswith("g1-run-incident-")

    clean = tmp_path / "g1-clean.mcap"
    _write(clean, [0.0, 0.2, 0.1], health=["inactive", "nominal", "nominal"])
    _event(clean, at_ns + 1_000_000_000)
    with pytest.raises(RecordingError, match="clean run"):
        check_recording(clean, no_incident=True)


def test_expect_incident_rejects_a_short_window(tmp_path: Path) -> None:
    fell = tmp_path / "g1-run.mcap"
    _write(fell, [0.0, 0.2], health=["incident", "incident"])
    _event(fell, 1_790_640_040 * 1_000_000_000, post_roll_s=2.0)
    with pytest.raises(RecordingError, match="not 10 s"):
        check_recording(fell, expect_incident=True)


def test_resolve_mcap_ignores_event_files(tmp_path: Path) -> None:
    telemetry = tmp_path / "run.mcap"
    telemetry.write_bytes(b"a")
    event = tmp_path / "events" / "run-incident-x.mcap"
    event.parent.mkdir()
    event.write_bytes(b"b")
    os.utime(event, (3_000, 3_000))
    assert resolve_mcap(tmp_path) == telemetry


def test_resolve_mcap_picks_the_newest_file(tmp_path: Path) -> None:
    older = tmp_path / "older.mcap"
    newer = tmp_path / "newer.mcap"
    older.write_bytes(b"a")
    newer.write_bytes(b"b")
    os.utime(older, (1_000, 1_000))
    os.utime(newer, (2_000, 2_000))
    assert resolve_mcap(tmp_path) == newer
    assert resolve_mcap(newer) == newer
