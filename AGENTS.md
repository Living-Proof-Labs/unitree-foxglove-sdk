# Agent guide

Read `docs/README.md` before changing this repository. It is the map. `README.md` is the human walkthrough and is not a substitute.

Remote: `https://github.com/Living-Proof-Labs/unitree-foxglove-sdk` (private). Do not add other remotes or force-push unless the user asks.

Upstream Unitree trees under `third_party/` are pinned submodules. Do not edit them. Behavior that has to differ from upstream lives in `src/unitree_foxglove/`.

`docs/decisions.md` records choices that were expensive to get right. Treat them as constraints, not as a list of options to revisit, unless the user is asking to change that behavior.

## Cursor Cloud specific instructions

Docker and Task are installed in the environment. The startup command starts the daemon. If `docker info` fails, run `sudo service docker start`. The image is `unitree-foxglove:local`; `task test` rebuilds it from cache and runs the unit tests inside it.

`task test` is the check that does not need a display. Ruff is not a Task target. The container user cannot write `/app/.ruff_cache`, so run `env UID="$(id -u)" GID="$(id -g)" docker compose run --rm --no-deps --entrypoint ruff sim check --no-cache src tests`.

Headless runs need `UNITREE_HEADLESS=1`. `task up` opens the MuJoCo window on `DISPLAY` (this desktop is `:1`). Run `xhost +local:` once per session before that window. Do not export `UNITREE_DOMAIN_ID` in the shell; Compose will pick it up and the unit tests' settings cases will disagree with `sim.env`. A device token is optional. Without `FOXGLOVE_DEVICE_TOKEN`, Foxlet stays off, which is what `task test` and the headless checks expect.

A healthy stack logs `simulator publishing`, `loco connected`, and `first sample received`, and `http://127.0.0.1:8770/g1_29dof.urdf` returns the G1 model. CycloneDDS saying `lo` is not multicast-capable is expected. The pretrained gait can still fall while the sim reports real-time factor `1.00`; that is the instability in `docs/operations.md`, and the MCAP under `recordings/` is the evidence. Do not loop `task e2e` until it passes.
