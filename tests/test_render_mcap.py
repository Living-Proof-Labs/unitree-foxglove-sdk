"""Validate replay pose pairing and portable PNG evidence without a display."""

import json
import struct
import zlib
from pathlib import Path

import foxglove
import numpy as np
import pytest
from foxglove.messages import JointState, JointStates, Pose, PoseInFrame, Quaternion, Vector3
from mcap.exceptions import McapError

from unitree_foxglove.convert import TOPIC_BASE_POSE, TOPIC_JOINT_STATES
from unitree_foxglove.render_mcap import read_frames, write_png


def test_replay_pairs_recorded_joints_with_pose_and_preserves_quaternion(tmp_path: Path) -> None:
    path = tmp_path / "telemetry.mcap"
    with foxglove.open_mcap(path):
        for index, value in enumerate((0.1, 0.8)):
            foxglove.log(
                TOPIC_JOINT_STATES,
                JointStates(joints=[JointState(name="left_knee_joint", position=value)]),
                log_time=index * 1_000_000_000,
            )
            foxglove.log(
                TOPIC_BASE_POSE,
                PoseInFrame(pose=Pose(
                    position=Vector3(x=index, z=0.8 - index * 0.6),
                    orientation=Quaternion(w=0.6, x=0.8),
                )),
                log_time=index * 1_000_000_000 + 1,
            )
            foxglove.log(
                "/health/state", {"state": "nominal" if index == 0 else "incident"},
                log_time=index * 1_000_000_000 + 2,
            )
    frames, report = read_frames(path)
    assert frames[0].joints == {"left_knee_joint": pytest.approx(0.1)}
    assert frames[1].joints == {"left_knee_joint": pytest.approx(0.8)}
    assert frames[-1].quaternion == pytest.approx((0.6, 0.8, 0.0, 0.0))
    assert report["minimum_base_height_m"] == pytest.approx(0.2)
    assert report["duration_s"] == 1.0
    assert report["health"] == ["nominal", "incident"]
    json.dumps(report)


def test_replay_rejects_event_only_recordings(tmp_path: Path) -> None:
    path = tmp_path / "event.mcap"
    with foxglove.open_mcap(path):
        foxglove.log("/event", {"kind": "incident"}, log_time=1)
    with pytest.raises(ValueError, match="no paired base poses"):
        read_frames(path)


def test_png_has_valid_checksums_and_exact_pixel_data(tmp_path: Path) -> None:
    pixels = np.array([[[255, 0, 0], [0, 255, 0]], [[0, 0, 255], [1, 2, 3]]], dtype=np.uint8)
    path = tmp_path / "frame.png"
    write_png(path, pixels)
    data = path.read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    offset = 8
    chunks = {}
    while offset < len(data):
        length = struct.unpack("!I", data[offset:offset + 4])[0]
        kind = data[offset + 4:offset + 8]
        payload = data[offset + 8:offset + 8 + length]
        crc = struct.unpack("!I", data[offset + 8 + length:offset + 12 + length])[0]
        assert crc == zlib.crc32(kind + payload) & 0xFFFFFFFF
        chunks[kind] = payload
        offset += 12 + length
    assert struct.unpack("!2I5B", chunks[b"IHDR"]) == (2, 2, 8, 2, 0, 0, 0)
    assert zlib.decompress(chunks[b"IDAT"]) == b"".join(b"\0" + row.tobytes() for row in pixels)


def test_replay_rejects_an_unfinished_recording(tmp_path: Path) -> None:
    path = tmp_path / "interrupted.mcap"
    path.write_bytes(b"\x89MCAP0\r\n" + b"\xff" * 32)
    with pytest.raises(McapError):
        read_frames(path)
