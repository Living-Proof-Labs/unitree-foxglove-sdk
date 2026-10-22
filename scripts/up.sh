#!/usr/bin/env bash
# sim, bridge, and loco. Foxlet joins when .env holds a device token.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"

# Create the recordings bind-mount as this user. If Docker creates it, it is root
# and the bridge cannot write MCAPs (PermissionError).
mkdir -p "$root/recordings"

services=(sim bridge loco)
profile=()
if grep -q '^FOXGLOVE_DEVICE_TOKEN=[^[:space:]]' .env 2>/dev/null; then
  profile=(--profile foxlet)
  services+=(foxlet)
fi

docker compose "${profile[@]}" up "${services[@]}"
