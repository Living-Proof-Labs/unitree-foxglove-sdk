#!/usr/bin/env bash
# Cloud Agent start: Docker for compose, X11 access for the MuJoCo viewer when present.
set -euo pipefail

if ! docker info >/dev/null 2>&1; then
  sudo service docker start
fi

if [[ -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null; then
  xhost +local:docker >/dev/null 2>&1 || true
fi
