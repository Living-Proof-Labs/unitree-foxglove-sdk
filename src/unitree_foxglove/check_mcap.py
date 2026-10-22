"""Read a recording and confirm the Foxglove channels contain motion.

The Foxglove SDK writes protobuf payloads and does not decode them. This uses
the schema embedded in the MCAP to rebuild the protobuf messages.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

from google.protobuf import descriptor_pb2
from google.protobuf.descriptor_pool import DescriptorPool
from google.protobuf.message_factory import GetMessageClass
from mcap.reader import make_reader

from unitree_foxglove.convert import (
    IMU_TOPICS,
    JOINT_TOPICS,
    POSE_TOPICS,
    TOPIC_BASE_POSE,
    TOPIC_SCAN,
)
from unitree_foxglove.health import INCIDENT, TOPIC_HEALTH_STATE
from unitree_foxglove.incidents import POST_ROLL_S, event_files_for, read_event_metadata
from unitree_foxglove.nav import STATE_ARRIVED, TOPIC_NAV_STATE

# Radians. Large enough to ignore sensor noise, small enough to catch a stand.
_MIN_JOINT_TRAVEL = 0.05
# Metres. A standing G1 pelvis is about 0.75 m up; a fallen one is under 0.4 m.
_MIN_STANDING_HEIGHT_M = 0.5


class RecordingError(Exception):
    """The MCAP is missing channels or the robot did not move."""


def main() -> None:
    """Check the MCAP path given as the first argument."""
    parser = argparse.ArgumentParser(description="Check a recording for motion.")
    parser.add_argument("path", type=Path, help="MCAP file or directory with the newest one")
    parser.add_argument(
        "--forward",
        type=float,
        default=None,
        metavar="METRES",
        help=f"also require the base to travel this far and stay above {_MIN_STANDING_HEIGHT_M} m",
    )
    parser.add_argument(
        "--no-incident",
        action="store_true",
        help=f"require {TOPIC_HEALTH_STATE} never reports {INCIDENT!r}, and no event MCAP",
    )
    parser.add_argument(
        "--expect-incident",
        action="store_true",
        help="require health to reach incident and exactly one event MCAP for this recording",
    )
    parser.add_argument(
        "--nav-arrived",
        action="store_true",
        help=f"require {TOPIC_SCAN} messages and {TOPIC_NAV_STATE} to end in {STATE_ARRIVED!r}",
    )
    args = parser.parse_args()
    if args.no_incident and args.expect_incident:
        parser.error("--no-incident and --expect-incident are alternatives")
    path = resolve_mcap(args.path)
    try:
        summary = check_recording(
            path,
            min_forward_m=args.forward,
            no_incident=args.no_incident,
            expect_incident=args.expect_incident,
            nav_arrived=args.nav_arrived,
        )
    except RecordingError as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1) from exc
    line = f"ok {path}: joints={summary['joint_messages']} travel={summary['joint_travel']:.3f} rad"
    if "forward" in summary:
        line += f" forward={summary['forward']:.3f} m min_z={summary['min_z']:.3f} m"
    if "health" in summary:
        line += f" health={summary['health']}"
    if "event" in summary:
        line += f" event={summary['event']}"
    if "nav" in summary:
        line += f" nav={summary['nav']} scans={summary['scans']}"
    print(line, flush=True)


def resolve_mcap(path: Path) -> Path:
    """Return ``path`` when it is a file, otherwise the newest MCAP in the directory."""
    if path.is_file():
        return path
    if not path.is_dir():
        raise RecordingError(f"Recording path does not exist: {path}")
    files = sorted(path.glob("*.mcap"), key=lambda item: item.stat().st_mtime)
    if not files:
        raise RecordingError(f"No MCAP files in {path}")
    return files[-1]


def check_recording(
    path: Path,
    min_forward_m: float | None = None,
    no_incident: bool = False,
    expect_incident: bool = False,
    nav_arrived: bool = False,
) -> dict:
    """Require joint, pose, and IMU messages, and joint travel above the threshold.

    With ``min_forward_m`` the base pose must also move that far in the ground
    plane between its first and last sample while never dropping below
    ``_MIN_STANDING_HEIGHT_M``, which is how the e2e tells walking from falling.
    With ``no_incident`` the health state must be recorded, never be
    ``incident``, and no event MCAP may exist for this file. With
    ``expect_incident`` health must reach ``incident`` and exactly one event
    MCAP must carry a window that ends 10 s after the fall. With
    ``nav_arrived`` the scan must be recorded and the navigator's last state
    must be ``arrived``.
    """
    counts: dict[str, int] = {}
    positions: list[list[float]] = []
    base_xyz: list[tuple[float, float, float]] = []
    health: list[str] = []
    nav: list[str] = []
    with path.open("rb") as handle:
        reader = make_reader(handle)
        summary = reader.get_summary()
        schemas = {} if summary is None else summary.schemas
        for _schema, channel, message in reader.iter_messages():
            counts[channel.topic] = counts.get(channel.topic, 0) + 1
            if channel.topic == TOPIC_HEALTH_STATE:
                state = json.loads(message.data)["state"]
                if not health or health[-1] != state:
                    health.append(state)
                continue
            if channel.topic == TOPIC_NAV_STATE:
                state = json.loads(message.data)["state"]
                if not nav or nav[-1] != state:
                    nav.append(state)
                continue
            schema = schemas.get(channel.schema_id)
            if schema is None:
                continue
            if channel.topic in JOINT_TOPICS:
                decoded = decode_payload(schema.name, schema.data, message.data)
                positions.append([joint.position for joint in decoded.joints])
            elif channel.topic == TOPIC_BASE_POSE and min_forward_m is not None:
                decoded = decode_payload(schema.name, schema.data, message.data)
                point = decoded.pose.position
                base_xyz.append((point.x, point.y, point.z))

    missing = [
        topic for topic in (*JOINT_TOPICS, *POSE_TOPICS, *IMU_TOPICS) if counts.get(topic, 0) == 0
    ]
    if missing:
        raise RecordingError(f"Missing messages on {', '.join(missing)}")
    if len(positions) < 2:
        raise RecordingError("Need at least two joint-state messages to judge motion")
    travel = _max_travel(positions)
    if travel < _MIN_JOINT_TRAVEL:
        raise RecordingError(f"Joint travel {travel:.4f} rad is below {_MIN_JOINT_TRAVEL} rad")
    result = {"joint_messages": float(len(positions)), "joint_travel": travel}
    if min_forward_m is not None:
        result.update(_check_walk(base_xyz, min_forward_m))
    if no_incident or expect_incident:
        result.update(_check_health(path, health, expect_incident=expect_incident))
    if nav_arrived:
        result.update(_check_nav(nav, counts.get(TOPIC_SCAN, 0)))
    return result


def _check_nav(nav: list[str], scans: int) -> dict:
    if scans == 0:
        raise RecordingError(f"Missing messages on {TOPIC_SCAN}")
    if not nav:
        raise RecordingError(f"Missing messages on {TOPIC_NAV_STATE}")
    timeline = " -> ".join(nav)
    if nav[-1] != STATE_ARRIVED:
        raise RecordingError(f"Navigator ended in {nav[-1]!r}, not {STATE_ARRIVED!r}: {timeline}")
    return {"nav": timeline, "scans": float(scans)}


def _check_health(path: Path, health: list[str], expect_incident: bool) -> dict:
    if not health:
        raise RecordingError(f"Missing messages on {TOPIC_HEALTH_STATE}")
    timeline = " -> ".join(health)
    events = event_files_for(path)
    fell = INCIDENT in health
    if expect_incident:
        if not fell:
            raise RecordingError(f"Health never reached incident: {timeline}")
        if len(events) != 1:
            raise RecordingError(f"Expected one incident event MCAP, found {len(events)}")
        _check_event_window(events[0])
        return {"health": timeline, "event": events[0].name}
    if fell:
        raise RecordingError(f"Health reported an incident: {timeline}")
    if events:
        names = ", ".join(event.name for event in events)
        raise RecordingError(f"Incident event MCAP written on a clean run: {names}")
    return {"health": timeline}


def _check_event_window(path: Path) -> None:
    try:
        metadata = read_event_metadata(path)
    except KeyError as exc:
        raise RecordingError(str(exc)) from exc
    if metadata.get("kind") != INCIDENT:
        raise RecordingError(f"{path.name} kind is {metadata.get('kind')!r}")
    try:
        at_ns = _ns_from_iso(metadata["at"])
        end_ns = _ns_from_iso(metadata["end"])
    except (KeyError, ValueError) as exc:
        raise RecordingError(f"{path.name} has no incident window: {exc}") from exc
    gap_s = (end_ns - at_ns) / 1_000_000_000
    if abs(gap_s - POST_ROLL_S) > 0.05:
        raise RecordingError(
            f"{path.name} ends {gap_s:.2f} s after the incident, not {POST_ROLL_S:.0f} s"
        )


def _ns_from_iso(text: str) -> int:
    moment = datetime.strptime(text, "%Y-%m-%dT%H:%M:%S.%fZ").replace(tzinfo=timezone.utc)
    return int(moment.timestamp() * 1_000_000_000)


def _check_walk(base_xyz: list[tuple[float, float, float]], min_forward_m: float) -> dict:
    if len(base_xyz) < 2:
        raise RecordingError("Need at least two base-pose messages to judge walking")
    (x0, y0, _), (x1, y1, _) = base_xyz[0], base_xyz[-1]
    forward = math.hypot(x1 - x0, y1 - y0)
    min_z = min(z for _, _, z in base_xyz)
    if min_z < _MIN_STANDING_HEIGHT_M:
        raise RecordingError(
            f"Base dropped to {min_z:.3f} m, below {_MIN_STANDING_HEIGHT_M} m: the robot fell"
        )
    if forward < min_forward_m:
        raise RecordingError(f"Base travelled {forward:.3f} m, below {min_forward_m} m")
    return {"forward": forward, "min_z": min_z}


def _max_travel(samples: list[list[float]]) -> float:
    width = min(len(sample) for sample in samples)
    travel = 0.0
    for index in range(width):
        column = [sample[index] for sample in samples]
        travel = max(travel, max(column) - min(column))
    return travel


_pools: dict[bytes, DescriptorPool] = {}


def decode_payload(schema_name: str, schema_data: bytes, payload: bytes) -> object:
    pool = _pools.get(schema_data)
    if pool is None:
        pool = DescriptorPool()
        descriptor = descriptor_pb2.FileDescriptorSet()
        descriptor.ParseFromString(schema_data)
        for file_proto in descriptor.file:
            pool.Add(file_proto)
        _pools[schema_data] = pool
    message_type = pool.FindMessageTypeByName(schema_name)
    message = GetMessageClass(message_type)()
    message.ParseFromString(payload)
    return message


if __name__ == "__main__":
    main()
