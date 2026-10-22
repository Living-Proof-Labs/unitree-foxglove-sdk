#!/usr/bin/env bash
# Idempotent Cloud Agent install: Docker daemon, recordings dir, compose image.
# bash treats UID as readonly, so pass host ids through env, never export UID.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is required" >&2
  exit 1
fi

if ! docker info >/dev/null 2>&1; then
  if [[ -x /usr/sbin/service ]]; then
    sudo service docker start || true
  fi
  for _ in $(seq 1 60); do
    docker info >/dev/null 2>&1 && break
    sleep 1
  done
  docker info >/dev/null
fi

if ! docker info >/dev/null 2>&1; then
  echo "dockerd did not become ready" >&2
  exit 1
fi

# Bind-mount target must exist as the agent user. If Docker creates it, it is root.
mkdir -p "$root/recordings"
if [[ ! -w "$root/recordings" ]]; then
  sudo chown -R "$(id -u):$(id -g)" "$root/recordings"
fi

if ! command -v task >/dev/null 2>&1; then
  sudo sh -c 'curl -fsSL https://taskfile.dev/install.sh | bash -s -- -d -b /usr/local/bin'
fi

env UID="$(id -u)" GID="$(id -g)" docker compose build
