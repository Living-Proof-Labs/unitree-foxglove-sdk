"""Write a review summary from retained simulation results."""

import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
lines = [f"### {path.name}", ""]
for label, file in (("Test", path / "run.json"), ("Replay rendering", path / "render/run.json")):
    if file.exists():
        run = json.loads(file.read_text())
        lines.append(f"- {label}: exit **{run['exit_code']}**, {run['seconds']:.1f} seconds")
    else:
        lines.append(f"- {label}: no result recorded (inspect build/setup logs)")
report = path / "replay.json"
if report.exists():
    for recording in json.loads(report.read_text()):
        lines.extend([
            "", f"Recording: `{recording['recording']}`",
            f"- Duration: {recording['duration_s']:.1f} seconds",
            f"- Minimum base height: {recording['minimum_base_height_m']:.3f} m",
            "- Health: " + " → ".join(recording['health']),
            "- Navigation: " + (" → ".join(recording['navigation']) or "not active"),
        ])
lines.extend(["", "Artifact includes telemetry/event MCAPs, replay PNGs, Compose logs,",
              "command logs, exit metadata, and asciinema v2 terminal casts."])
text = "\n".join(lines) + "\n"
path.mkdir(parents=True, exist_ok=True)
(path / "summary.md").write_text(text)
print(text, end="")
