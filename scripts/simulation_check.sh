#!/usr/bin/env bash
# Run the same developer task and preserve evidence even when acceptance fails.
set -euo pipefail
case "${1:?usage: simulation_check.sh fall|walk|hallway}" in
  fall) run=e2e-fall; check=e2e-fall; world=open ;;
  walk) run=e2e; check=e2e; world=open ;;
  hallway) run=e2e-hallway; check=e2e-hallway; world=hallway10 ;;
  *) exit 64 ;;
esac
result=0
python3 scripts/capture.py "artifacts/$run" -- task "$check" || result=$?
render_result=0
python3 scripts/capture.py "artifacts/$run/render" -- task artifacts:render RUN="$run" WORLD="$world" || render_result=$?
if [[ "$result" -ne 0 ]]; then exit "$result"; fi
exit "$render_result"
