#!/usr/bin/env bash
# Headless G1, two phases. First a walk that must stay up. Then a deliberate fall
# with the policy off, which must record health incident and one event MCAP.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"

# shellcheck source=host_prep.sh
source "$root/scripts/host_prep.sh"
prepare_recordings "$root/recordings"

# bash treats UID as readonly, so pass the host ids on each docker compose command.
host_uid="$(id -u)"
host_gid="$(id -g)"
export UNITREE_ROBOT=g1
export UNITREE_HEADLESS=1
export UNITREE_DOMAIN_ID="${UNITREE_DOMAIN_ID:-7}"
# Ports off the defaults so the e2e run can coexist with a running `task up`.
export FOXGLOVE_PORT="${FOXGLOVE_PORT:-8766}"
export ASSETS_PORT="${ASSETS_PORT:-8771}"
export RECORDINGS_DIR=/data/recordings/e2e
export UNITREE_RECORD_ROLL_S=0
# No Remote Access gateway: the token in .env belongs to the interactive stack.
export FOXGLOVE_DEVICE_TOKEN=
min_forward_m="${E2E_MIN_FORWARD_M:-0.3}"

project="unitree-foxglove-e2e"
output_dir="$root/recordings/e2e"
rm -rf "$output_dir"
mkdir -p "$output_dir"

compose() {
  env UID="$host_uid" GID="$host_gid" docker compose -p "$project" "$@"
}

cleanup() {
  compose down --remove-orphans >/dev/null 2>&1 || true
}
trap cleanup EXIT

compose up -d --build sim bridge loco

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
  -v "$output_dir:/data/recordings/e2e:ro" \
  sim -m unitree_foxglove.check_mcap /data/recordings/e2e --forward "$min_forward_m" --no-incident

echo "deliberate fall, walking policy off"
compose start bridge
wait_for_log bridge "first sample received" 60 2

compose run --rm --no-deps --entrypoint python bridge -m unitree_foxglove.ws_client \
  --url "ws://127.0.0.1:${FOXGLOVE_PORT}" \
  --fall \
  --band-lowers "${E2E_BAND_LOWERS:-7}" \
  --settle "${E2E_SETTLE:-3}" \
  --after-release "${E2E_AFTER_RELEASE:-20}"

sleep 1
compose stop bridge

compose run --rm --no-deps --entrypoint python \
  -v "$output_dir:/data/recordings/e2e:ro" \
  sim -m unitree_foxglove.check_mcap /data/recordings/e2e --expect-incident
