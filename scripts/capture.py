"""Run a command and retain its exit status, log, and asciinema v2 terminal cast."""

from __future__ import annotations

import argparse
import codecs
import json
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command
    if command[:1] == ["--"]:
        command = command[1:]
    if not command:
        parser.error("a command is required after the output directory")
    args.directory.mkdir(parents=True, exist_ok=True)
    started = datetime.now(timezone.utc)
    before = time.monotonic()
    metadata = {"command": command, "started_utc": started.isoformat()}
    with (
        (args.directory / "task.log").open("wb") as log,
        (args.directory / "terminal.cast").open("w") as cast,
    ):
        cast.write(json.dumps({
            "version": 2, "width": 120, "height": 40,
            "timestamp": int(started.timestamp()), "title": " ".join(command),
        }) + "\n")
        decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
        with subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT) as proc:
            assert proc.stdout is not None
            while chunk := os.read(proc.stdout.fileno(), 65536):
                log.write(chunk)
                log.flush()
                sys.stdout.buffer.write(chunk)
                sys.stdout.buffer.flush()
                output = decoder.decode(chunk)
                if output:
                    cast.write(json.dumps([
                        time.monotonic() - before, "o", output.replace("\n", "\r\n"),
                    ]) + "\n")
                    cast.flush()
            code = proc.wait()
    code = code if code >= 0 else 128 - code
    metadata.update(exit_code=code, seconds=round(time.monotonic() - before, 3))
    (args.directory / "run.json").write_text(json.dumps(metadata, indent=2) + "\n")
    raise SystemExit(code)


if __name__ == "__main__":
    main()
