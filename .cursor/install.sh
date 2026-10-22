#!/usr/bin/env bash
# Cloud Agent install: prebuild the MuJoCo + Foxglove stack image (idempotent).
set -euo pipefail
cd "$(dirname "$0")/.."

if ! command -v docker >/dev/null; then
  echo "docker is required but not installed" >&2
  exit 1
fi

if ! docker info >/dev/null 2>&1; then
  sudo service docker start
  for _ in $(seq 1 30); do
    docker info >/dev/null 2>&1 && break
    sleep 1
  done
fi

if ! command -v task >/dev/null; then
  curl -fsSL https://taskfile.dev/install.sh | sh -s -- -d -b /usr/local/bin
fi

host_uid="$(id -u)"
host_gid="$(id -g)"
env UID="$host_uid" GID="$host_gid" docker compose build
