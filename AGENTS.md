# Agent guide

Read `docs/README.md` before changing this repository. It is the map. `README.md` is the human walkthrough and is not a substitute.

Remote: `https://github.com/Living-Proof-Labs/unitree-foxglove-sdk` (private). Do not add other remotes or force-push unless the user asks.

Upstream Unitree trees under `third_party/` are pinned submodules. Do not edit them. Behavior that has to differ from upstream lives in `src/unitree_foxglove/`.

`docs/decisions.md` records choices that were expensive to get right. Treat them as constraints, not as a list of options to revisit, unless the user is asking to change that behavior.

## Cursor Cloud specific instructions

Docker starts on agent boot (see `.cursor/start.sh`). The environment build pre-runs `.cursor/install.sh`, which produces `unitree-foxglove:local` via `docker compose build`.

**Quick verification (no GUI):**

- `task test` — unit tests in the image.
- `UNITREE_HEADLESS=1 UNITREE_DURATION=15 env UID=$(id -u) GID=$(id -g) docker compose up sim` — MuJoCo stepping headlessly; sim log should show `real-time factor 1.00`.
- `task e2e` — full headless stand-up, walk, MCAP check, then deliberate fall. The walking policy falls on some runs even at RTF 1.00; that is documented in `docs/operations.md` and is not necessarily an environment fault. Do not loop `task e2e` until it passes.

**Interactive stack (VNC desktop, `DISPLAY=:1`):**

- `xhost +local:docker` once per session if the MuJoCo window fails to open.
- `task up` — sim window, bridge on `ws://localhost:8765`, and walking policy. Use `task down` to stop.

**E2E tuning:** `E2E_SETTLE`, `E2E_SECONDS`, and `E2E_PAUSE_SECONDS` (see `scripts/e2e.sh`) adjust stand-up timing. The e2e compose project uses domain 7 and ports 8766/8771 so it can run beside a `task up` on domain 1.

Do not put `FOXGLOVE_DEVICE_TOKEN` in tracked files. Foxlet is optional and only for upload demos with `.env`.
