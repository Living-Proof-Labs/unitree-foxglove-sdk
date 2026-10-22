#!/usr/bin/env bash
# Boot-time start: Docker, X11 for the MuJoCo window, then sim + bridge + loco.
# Idempotent. bash treats UID as readonly; pass host ids through env only.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"

if ! docker info >/dev/null 2>&1; then
  if [[ -x /usr/sbin/service ]]; then
    sudo service docker start || true
  fi
fi

if ! docker info >/dev/null 2>&1; then
  if ! pgrep -x dockerd >/dev/null 2>&1; then
    sudo mkdir -p /var/run
    sudo nohup dockerd >/tmp/dockerd.log 2>&1 &
  fi
fi

for _ in $(seq 1 60); do
  if docker info >/dev/null 2>&1; then
    break
  fi
  sleep 1
done
if ! docker info >/dev/null 2>&1; then
  echo "dockerd did not become ready" >&2
  sudo tail -n 80 /tmp/dockerd.log 2>/dev/null || true
  exit 1
fi

mkdir -p "$root/recordings"
if [[ ! -w "$root/recordings" ]]; then
  sudo chown -R "$(id -u):$(id -g)" "$root/recordings"
fi

headless=0
if [[ -z "${DISPLAY:-}" && -S /tmp/.X11-unix/X1 ]]; then
  export DISPLAY=:1
fi
if [[ -n "${DISPLAY:-}" ]]; then
  for _ in $(seq 1 30); do
    if xdpyinfo >/dev/null 2>&1; then
      break
    fi
    sleep 1
  done
fi
if [[ -n "${DISPLAY:-}" ]] && xdpyinfo >/dev/null 2>&1; then
  xhost +local:docker >/dev/null 2>&1 || xhost +local: >/dev/null 2>&1 || true
else
  headless=1
  echo "no X11 display; starting the simulator headless"
fi
export UNITREE_HEADLESS="$headless"

compose() {
  env UID="$(id -u)" GID="$(id -g)" UNITREE_HEADLESS="$UNITREE_HEADLESS" docker compose "$@"
}

compose up -d sim bridge loco

for _ in $(seq 1 90); do
  if compose logs bridge 2>/dev/null | grep -q "first sample received"; then
    echo "G1 stack is up (Foxglove ws://127.0.0.1:8765)"
    exit 0
  fi
  sleep 1
done

echo "bridge did not log 'first sample received'" >&2
compose ps >&2 || true
compose logs --tail=80 >&2 || true
exit 1
