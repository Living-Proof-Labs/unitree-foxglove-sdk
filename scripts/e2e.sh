#!/usr/bin/env bash
# Headless G1, two phases. First a walk that must stay up. Then a deliberate fall
# with the policy off, which must record health incident and one event MCAP.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"

# bash treats UID as readonly, so pass the host ids on each docker compose command.
host_uid="$(id -u)"
host_gid="$(id -g)"
export UNITREE_ROBOT=g1
export UNITREE_HEADLESS=1
export UNITREE_DOMAIN_ID="${UNITREE_DOMAIN_ID:-7}"
# Ports off the defaults so the e2e run can coexist with a running `task up`.
export FOXGLOVE_PORT="${FOXGLOVE_PORT:-8766}"
export ASSETS_PORT="${ASSETS_PORT:-8771}"
case "${E2E_MODE:-all}" in
  all) run_name=e2e ;;
  fall) run_name=e2e-fall ;;
  *) echo "E2E_MODE must be all or fall" >&2; exit 64 ;;
esac
export RECORDINGS_DIR="/data/recordings/$run_name"
export UNITREE_RECORD_ROLL_S=0
# No Remote Access gateway: the token in .env belongs to the interactive stack.
export FOXGLOVE_DEVICE_TOKEN=
min_forward_m="${E2E_MIN_FORWARD_M:-0.3}"

project="unitree-foxglove-e2e"
output_dir="$root/recordings/$run_name"
rm -rf "$output_dir"
mkdir -p "$output_dir"

compose() {
  env UID="$host_uid" GID="$host_gid" docker compose -p "$project" "$@"
}

cleanup() {
  local result=$?
  trap - EXIT
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

echo "waiting for the bridge and the walking policy"
wait_for_log bridge "first sample received"
wait_for_log loco "loco connected"

if [[ "${E2E_MODE:-all}" != fall ]]; then
  # Same path the Teleop and Service Call panels use: a Foxglove WebSocket client.
  compose run --rm --no-deps --entrypoint python bridge -m unitree_foxglove.ws_client \
    --url "ws://127.0.0.1:${FOXGLOVE_PORT}" \
    --band-lowers "${E2E_BAND_LOWERS:-7}" \
    --speed "${E2E_SPEED:-0.4}" \
    --seconds "${E2E_SECONDS:-4}" \
    --settle "${E2E_SETTLE:-3}"

  sleep 1
  compose logs sim | grep "real-time" | tail -n 3
  compose stop bridge
  compose logs loco | tail -n 12

  compose run --rm --no-deps --entrypoint python \
    -v "$output_dir:/data/recordings/$run_name:ro" \
    sim -m unitree_foxglove.check_mcap "/data/recordings/$run_name" --forward "$min_forward_m" --no-incident

  echo "restarting the bridge for the deliberate fall"
  compose start bridge
  wait_for_log bridge "first sample received" 60 2
fi

echo "deliberate fall, walking policy off"

compose run --rm --no-deps --entrypoint python bridge -m unitree_foxglove.ws_client \
  --url "ws://127.0.0.1:${FOXGLOVE_PORT}" \
  --fall \
  --band-lowers "${E2E_BAND_LOWERS:-7}" \
  --settle "${E2E_SETTLE:-3}" \
  --after-release "${E2E_AFTER_RELEASE:-20}"

sleep 1
compose stop bridge

compose run --rm --no-deps --entrypoint python \
  -v "$output_dir:/data/recordings/$run_name:ro" \
  sim -m unitree_foxglove.check_mcap "/data/recordings/$run_name" --expect-incident
