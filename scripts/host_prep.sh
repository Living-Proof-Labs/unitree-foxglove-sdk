#!/usr/bin/env bash
# Host steps shared by task up and the headless checks.
#
# Compose bind-mounts recordings/. When that path is missing, the Docker daemon
# creates it as root, and the container (the host uid) cannot open the MCAP.
# The viewer also needs the host X server to accept the container's connection.

prepare_recordings() {
  local dir="$1"
  if [[ -d "$dir" && -w "$dir" ]]; then
    return 0
  fi
  mkdir -p "$dir" 2>/dev/null || true
  if [[ -w "$dir" ]]; then
    return 0
  fi
  sudo -n chown "$(id -u):$(id -g)" "$dir"
  mkdir -p "$dir"
}

allow_local_display() {
  if [[ -z "${DISPLAY:-}" ]] || ! command -v xhost >/dev/null 2>&1; then
    return 0
  fi
  xhost +local: >/dev/null 2>&1 || true
}
