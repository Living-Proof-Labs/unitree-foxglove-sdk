"""Render review screenshots from recorded robot poses and joint states, offline."""

from __future__ import annotations

import argparse
import json
import os
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from mcap.exceptions import McapError
from mcap.reader import make_reader

from unitree_foxglove.check_mcap import decode_payload
from unitree_foxglove.convert import TOPIC_BASE_POSE, TOPIC_JOINT_STATES
from unitree_foxglove.robots import G1
from unitree_foxglove.worlds import build_model, world_for


@dataclass
class Frame:
    time_ns: int
    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
    joints: dict[str, float]


def read_frames(path: Path) -> tuple[list[Frame], dict]:
    frames = []
    joints: dict[str, float] = {}
    states: dict[str, list[str]] = {"/health/state": [], "/nav/state": []}
    with path.open("rb") as handle:
        for schema, channel, message in make_reader(handle).iter_messages():
            if channel.topic in states:
                state = json.loads(message.data)["state"]
                timeline = states[channel.topic]
                if not timeline or timeline[-1] != state:
                    timeline.append(state)
            elif channel.topic in (TOPIC_BASE_POSE, TOPIC_JOINT_STATES) and schema:
                decoded = decode_payload(schema.name, schema.data, message.data)
                if channel.topic == TOPIC_JOINT_STATES:
                    joints = {joint.name: joint.position for joint in decoded.joints}
                elif joints:
                    p, q = decoded.pose.position, decoded.pose.orientation
                    frames.append(Frame(
                        message.log_time, (p.x, p.y, p.z), (q.w, q.x, q.y, q.z), joints,
                    ))
    if not frames:
        raise ValueError(f"{path}: no paired base poses and joint states to render")
    return frames, {
        "recording": path.name,
        "frames": len(frames),
        "duration_s": (frames[-1].time_ns - frames[0].time_ns) / 1e9,
        "minimum_base_height_m": min(frame.position[2] for frame in frames),
        "health": states["/health/state"],
        "navigation": states["/nav/state"],
    }


def write_png(path: Path, pixels: np.ndarray) -> None:
    """Write RGB renderer output without an additional image dependency."""
    height, width, channels = pixels.shape
    if channels != 3 or pixels.dtype != np.uint8:
        raise ValueError("PNG input must be an RGB uint8 array")

    def chunk(kind: bytes, data: bytes) -> bytes:
        crc = zlib.crc32(kind + data) & 0xFFFFFFFF
        return struct.pack("!I", len(data)) + kind + data + struct.pack("!I", crc)

    rows = b"".join(b"\0" + row.tobytes() for row in pixels)
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack("!2I5B", width, height, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(rows))
        + chunk(b"IEND", b"")
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--world", default="open")
    args = parser.parse_args()
    files = sorted(args.directory.glob("*.mcap"))
    if not files:
        parser.error(f"no telemetry MCAP files in {args.directory}")
    # Import after the caller has selected the software rendering backend.
    import mujoco

    root = Path(os.environ.get("UNITREE_MUJOCO_ROOT", "third_party/unitree_mujoco"))
    model = build_model(root / "unitree_robots/g1" / G1.scene_file, world_for(args.world))
    data = mujoco.MjData(model)
    joint_addresses = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i): model.jnt_qposadr[i]
        for i in range(model.njnt)
        if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE
    }
    # Make hallway walls translucent in the review view so they do not hide the robot.
    for i in range(model.ngeom):
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or "").startswith("wall_"):
            model.geom_rgba[i, 3] = 0.2
    reports = []
    with mujoco.Renderer(model, height=480, width=640) as renderer:
        for path in files:
            try:
                frames, report = read_frames(path)
            except (McapError, ValueError, EOFError) as exc:
                reports.append({"recording": path.name, "error": str(exc)})
                continue
            selected = {
                "start": 0, "midpoint": len(frames) // 2, "end": len(frames) - 1,
                "lowest": min(range(len(frames)), key=lambda i: frames[i].position[2]),
            }
            directory = args.directory / "screenshots" / path.stem
            directory.mkdir(parents=True, exist_ok=True)
            report["screenshots"] = {}
            for label, index in selected.items():
                frame = frames[index]
                mujoco.mj_resetData(model, data)
                data.qpos[:3] = frame.position
                data.qpos[3:7] = frame.quaternion
                for name, value in frame.joints.items():
                    if name in joint_addresses:
                        data.qpos[joint_addresses[name]] = value
                mujoco.mj_forward(model, data)
                camera = mujoco.MjvCamera()
                camera.lookat[:] = (*frame.position[:2], max(0.5, frame.position[2]))
                camera.distance, camera.azimuth, camera.elevation = 3.0, 135.0, -20.0
                renderer.update_scene(data, camera=camera)
                write_png(directory / f"{label}.png", renderer.render())
                report["screenshots"][label] = {"log_time_ns": frame.time_ns, "frame": index}
            reports.append(report)
    (args.directory / "replay.json").write_text(json.dumps(reports, indent=2) + "\n")
    errors = [report for report in reports if "error" in report]
    print(f"Rendered {len(reports) - len(errors)} recording(s) into "
          f"{args.directory / 'screenshots'}")
    for report in errors:
        print(f"Could not replay {report['recording']}: {report['error']}")
    if errors:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
