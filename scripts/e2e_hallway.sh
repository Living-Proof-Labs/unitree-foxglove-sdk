#!/usr/bin/env bash
# Headless G1 in a closed hallway. The navigator must walk it to the end wall,
# stop there, and the robot must stay up the whole way. E2E_WORLD picks the
# hallway (hallway10 by default; hallway20 takes about twice as long).
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"

host_uid="$(id -u)"
host_gid="$(id -g)"
export UNITREE_ROBOT=g1
export UNITREE_HEADLESS=1
export UNITREE_WORLD="${E2E_WORLD:-hallway10}"
export UNITREE_DOMAIN_ID="${UNITREE_DOMAIN_ID:-7}"
export FOXGLOVE_PORT="${FOXGLOVE_PORT:-8766}"
export ASSETS_PORT="${ASSETS_PORT:-8771}"
export RECORDINGS_DIR=/data/recordings/e2e-hallway
export UNITREE_RECORD_ROLL_S=0
# No Remote Access gateway: the token in .env belongs to the interactive stack.
export FOXGLOVE_DEVICE_TOKEN=

case "$UNITREE_WORLD" in
  hallway10) default_forward=6.0; default_timeout=120 ;;
  hallway20) default_forward=15.0; default_timeout=240 ;;
  *) echo "E2E_WORLD must be hallway10 or hallway20" >&2; exit 64 ;;
esac
min_forward_m="${E2E_MIN_FORWARD_M:-$default_forward}"
nav_timeout_s="${E2E_NAV_TIMEOUT:-$default_timeout}"

project="unitree-foxglove-e2e"
output_dir="$root/recordings/e2e-hallway"
rm -rf "$output_dir"
mkdir -p "$output_dir"

compose() {
  env UID="$host_uid" GID="$host_gid" docker compose -p "$project" "$@"
}

cleanup() {
  local result=$?
  trap - EXIT
  # A stuck native SDK call may ignore SIGTERM; retain Python thread stacks first.
  if [[ "$result" -ne 0 ]]; then
    compose kill -s SIGUSR1 bridge >/dev/null 2>&1 || true
  fi
  # Close MCAPs and collect diagnostics before removing containers, even on failure.
  compose stop bridge >/dev/null 2>&1 || true
  compose logs --no-color > "$output_dir/compose.log" 2>&1 || true
  compose ps -a > "$output_dir/compose-ps.txt" 2>&1 || true
  compose down --remove-orphans >/dev/null 2>&1 || true
  exit "$result"
}
trap cleanup EXIT

# Task's :with-build variant builds before invoking this script.
compose up -d --no-build sim bridge loco

wait_for_log() {
  local service="$1" pattern="$2" tries="${3:-60}" need="${4:-1}"
  for _ in $(seq 1 "$tries"); do
    local seen
    seen="$(compose logs "$service" 2>/dev/null | grep -c "$pattern" || true)"
    if [[ "$seen" -ge "$need" ]]; then
      return 0
    fi
    sleep 1
  done
  compose logs
  echo "$service did not log '$pattern' $need time(s)" >&2
  return 1
}

echo "waiting for the bridge and the walking policy ($UNITREE_WORLD)"
wait_for_log bridge "first sample received"
wait_for_log bridge "world $UNITREE_WORLD"
wait_for_log loco "loco connected"

compose run --rm --no-deps --entrypoint python bridge -m unitree_foxglove.ws_client \
  --url "ws://127.0.0.1:${FOXGLOVE_PORT}" \
  --nav \
  --nav-timeout "$nav_timeout_s" \
  --band-lowers "${E2E_BAND_LOWERS:-7}" \
  --settle "${E2E_SETTLE:-3}"

sleep 1
compose logs sim | grep "real-time" | tail -n 3
compose logs bridge | grep "nav " | tail -n 5
compose stop bridge

compose run --rm --no-deps --entrypoint python \
  -v "$output_dir:/data/recordings/e2e-hallway:ro" \
  sim -m unitree_foxglove.check_mcap /data/recordings/e2e-hallway \
  --forward "$min_forward_m" --no-incident --nav-arrived
