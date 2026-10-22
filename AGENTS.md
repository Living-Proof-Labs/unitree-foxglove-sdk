# Agent guide

Read `docs/README.md` before changing this repository. It is the map. `README.md` is the human walkthrough and is not a substitute.

Remote: `https://github.com/Living-Proof-Labs/unitree-foxglove-sdk` (private). Do not add other remotes or force-push unless the user asks.

Upstream Unitree trees under `third_party/` are pinned submodules. Do not edit them. Behavior that has to differ from upstream lives in `src/unitree_foxglove/`.

`docs/decisions.md` records choices that were expensive to get right. Treat them as constraints, not as a list of options to revisit, unless the user is asking to change that behavior.
