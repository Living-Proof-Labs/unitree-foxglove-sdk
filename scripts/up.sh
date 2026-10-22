#!/usr/bin/env bash
# sim, bridge, and loco. Foxlet joins when .env holds a device token.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"

services=(sim bridge loco)
profile=()
if grep -q '^FOXGLOVE_DEVICE_TOKEN=[^[:space:]]' .env 2>/dev/null; then
  profile=(--profile foxlet)
  services+=(foxlet)
fi

docker compose "${profile[@]}" up "${services[@]}"
