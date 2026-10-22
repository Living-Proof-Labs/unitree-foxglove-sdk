# Agent guide

Read `docs/README.md` before changing this repository. It is the map. `README.md` is the human walkthrough and is not a substitute.

Remote: `https://github.com/Living-Proof-Labs/unitree-foxglove-sdk` (private). Do not add other remotes or force-push unless the user asks.

Upstream Unitree trees under `third_party/` are pinned submodules. Do not edit them. Behavior that has to differ from upstream lives in `src/unitree_foxglove/`.

`docs/decisions.md` records choices that were expensive to get right. Treat them as constraints, not as a list of options to revisit, unless the user is asking to change that behavior.

## Cursor Cloud

The Cloud Agent VM has Docker, Task, and a VNC desktop on `DISPLAY=:1`. Boot starts Docker, allows Docker to use that X server, and brings up `sim`, `bridge`, and `loco` so `ws://127.0.0.1:8765` is already live.

- Do not `export UID=...`. bash treats `UID` as readonly. Task sets it; otherwise prefix compose with `env UID="$(id -u)" GID="$(id -g)"`.
- `recordings/` must exist and be writable by the agent user before compose starts. `scripts/up.sh` and the Cloud start script create it. If the bridge dies with `PermissionError` on an MCAP, `sudo chown -R "$(id -u):$(id -g)" recordings` and restart `bridge`.
- For a viewer window: `DISPLAY=:1 xhost +local:docker` then `task up`. Headless: `UNITREE_HEADLESS=1`.
- Drive the live stack with `env UID="$(id -u)" GID="$(id -g)" docker compose run --rm --no-deps --entrypoint python bridge -m unitree_foxglove.ws_client`.
- `task e2e` uses domain 7 and ports 8766/8771, so it can coexist with `task up`.
