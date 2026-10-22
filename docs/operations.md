# Operations

## Validate Pause and Resume

Pause keeps the walking policy balancing at zero velocity and ignores Teleop. Resume lets Teleop drive again. The robot never leaves the policy, so there is nothing to recover from.

Restart so the bridge picks up the services. In the terminal where `task up` is attached, press Ctrl+C, then:

```bash
task up
```

Import `foxglove/g1-layout.json` again. Foxglove keeps the layout you already imported, so an old one still shows **Stop (crouch)**. The new one has **Pause** and **Resume**.

1. Stand, start the policy, lower the rope about seven times, and release it, as in the root README. Drive a little with Teleop.
2. Press **Pause**. The response says `paused`, and `/loco/mode` reads `paused`. The robot stops travelling. It keeps shifting its feet in place; that is the policy at zero speed, not a fault.
3. Hold a Teleop direction for a few seconds. The robot does not travel. The velocity plot stays near zero, and `/cmd_vel` in the plot does not rise because paused Twists are not logged as commands.
4. Press **Resume**. `/loco/mode` reads `walk`. Teleop drives it again.
5. Throughout, pelvis height stays near 0.75 m. A drop toward 0.1 m is a fall and a failed check.

**Pause** before **2. Start policy** answers `not walking, nothing to pause` and does nothing. **Damp (motors off)** is still the collapse.

## Validate the health state

Health watches, and the only thing it does is damp the motors when a fall becomes `incident`. The layout shows it three ways: the **Indicator** at the top right (colour and label), the **Health and loco mode** State Transitions strip under the plots, and the raw `/health/state` message next to the Indicator. Restart `task up` and re-import `foxglove/g1-layout.json` as above.

1. Before Stand, while hanging: `inactive`, reason `hanging on the elastic band`.
2. Stand, start, lower, release. The strip switches to `nominal`, reason `upright, walk`.
3. Drive, Pause, Resume. It stays `nominal`; the reason follows the mode (`upright, paused`).
4. Make it fall. **Reset sim**, **1. Stand**, lower the rope about seven times, then **4. Release band** without **2. Start policy**. Nothing balances, so it tips. The first half second reads `degraded` (the hold that separates a stumble from a fall), then `incident` with the pelvis height or tilt in the reason. The Indicator turns red, the bridge logs `health incident: motors damped`, and `/loco/mode` reads `damp`. The legs stop kicking. The next health sample is `inactive`, reason `motors damped`: a damped robot is not judged, and that is the confirmation the policy stopped. The incident and the event file are already recorded. Holding a Teleop arrow does not start it again.
5. **Reset sim**. Hanging on the rope reads `inactive` again. **Damp** while the robot is upright does the same, reason `motors damped`.
6. Stop `task up` with Ctrl+C while the layout is open. Before the WebSocket closes, the strip may show `offline`.

## Validate incident events

Continue from the fall in the health steps above. Release without the walking policy, then wait. The window closes 10 s after health enters `incident`, not when you press Damp.

1. About 10 s after the Indicator turns red, the bridge log prints `incident event g1-<UTC>-incident-<UTC>.mcap`. The file is `recordings/events/`. It is a few kilobytes. Stopping `task up` before those 10 s also writes the file, with a shorter `end`.
2. Open that file in Foxglove. It has no plots. The metadata record `event` has `start`, `end`, `at`, and `reason`. `end` is 10 s after `at` when the bridge was left running.
3. Open the telemetry MCAP from the same run and seek to `at`. Health is `incident` there. The lead-up is the 30 s before `at`, in the telemetry file, not in the event file.

`task e2e` does this fall itself, after a walk that must stay up. A walk that never goes `degraded` or `incident` writes no file under `events/`. A wall bump that recovers writes one `*-degraded-*.mcap` about 3 s after health returns to `nominal`. Creating the Foxglove event is the `foxglove-event-trigger` project, which is not in this repo and needs no key here.

`task up` also starts Foxlet when `.env` has `FOXGLOVE_DEVICE_TOKEN`. Its log should show the device connected. Event files then appear on [g1-sim](https://app.foxglove.dev/anybotics/p/prj_0edsooXR5WbJ2CWJ/devices/dev_0edswdPBDBYO4uxe), in the Experimental project of the ANYbotics Foxglove org, without an import click. Telemetry files appear as not imported. The event creator is what imports the ones that overlap an event. Telemetry rolls every 60 s, so each of those files is about a minute. `task down` stops Foxlet too. The headless checks do not start it, and they keep a single telemetry file.

## Validate the hallway walk

The navigator follows the centre of a hallway it has no map of and stops in front of the wall that closes it. It runs only in a hallway world; in the open world `nav/start` sees no walls and ends at once with `blind`.

```bash
task hallway                   # 10 m
task hallway WORLD=hallway20   # 20 m
```

Re-import `foxglove/g1-layout.json` if the right-hand column has no **5. Walk the hallway** button and no `nav` Indicator.

1. The 3D panel shows four beige walls on `/world/scene` and, once the robot is up, a ring of coloured points on `/scan`. The bridge log prints `world hallway10: 10 m hallway drawn on /world/scene`. Rays that hit nothing draw nothing; in the 20 m hallway the far wall is beyond the 10 m range at the start.
2. Stand, start, lower seven times, release. Wait until the robot is steady. The gait drifts in yaw while it settles, so it rarely faces straight down the hallway; that is what the alignment step is for.
3. **5. Walk the hallway**. The response says `walking`. The `nav` Indicator reads `aligning` while it turns to face the far end, then `walking`. `/nav/state` shows `front_m` falling and `left_m`, `right_m` near 1.0 when it is centred. `heading_deg` is the corridor heading relative to the robot; it should settle near zero.
4. Below 3.5 m the commanded `vx` ramps down. At about 1 m the Indicator turns green, `ARRIVED`, and the bridge log prints `nav arrived: wall 0.98 m ahead` or thereabouts. The robot keeps stepping in place; that is the policy at zero velocity, exactly as after Pause.
5. Teleop, **Pause**, and the rest work as before. Pressing a Teleop arrow during the walk stops navigation with `stopped` and drives the robot by hand.

Before **2. Start policy**, `nav/start` answers `start the walking policy first (loco/start), then nav/start` and does nothing. During **Pause** it is also refused.

If the robot falls on the way, health reaches `incident`, navigation stops, the motors damp, and `/nav/state` reads `stopped`. Navigation did not cause it: see the gait note under [decisions.md](decisions.md#locomotion). **Reset sim** and stand it up again. The policy stays off until **Start policy**.

`task e2e-hallway` runs the same walk headlessly and requires `arrived`. `WORLD=hallway20` allows 240 s and requires 15 m of travel. When it fails with `navigator ended in 'stopped'`, the bridge lines printed above the failure show the last decisions, and `recordings/e2e-hallway/` holds the MCAP.

## Commands

From the repository root, with Docker and Task:

| Task | What it does |
| --- | --- |
| `task up` | Build if needed, then sim, bridge, and loco in the foreground. Leave it attached. Ctrl+C stops all three, and the WebSocket goes away with them. |
| `task sim` | Viewer only. No policy, no Foxglove. |
| `task foxglove` | Prints the layout-import hint and opens the web app on `ws://localhost:8765`. Does not start the stack and does not select a layout. |
| `task test` | Unit tests inside the image. |
| `task hallway` | Same as `task up` in a closed hallway. `WORLD=hallway10` (default) or `hallway20`. |
| `task e2e` | Headless walk that must stay up, then a deliberate fall that must write one event MCAP. Domain 7. |
| `task e2e-hallway` | Headless navigator that must reach `arrived` without a fall. Same `WORLD` variable, same domain and ports as `task e2e`. |
| `task down` | `docker compose down --remove-orphans`. |

The same stand-up the panels perform, while `task up` is running:

```bash
docker compose run --rm --no-deps --entrypoint python bridge -m unitree_foxglove.ws_client
```

X11: `xhost +local:docker` once per login, and `DISPLAY` must be the host session. Headless sets `UNITREE_HEADLESS=1` and needs neither.

## What "healthy" looks like

Bridge log contains `live Foxglove connection ws://127.0.0.1:8765`, then `robot model http://127.0.0.1:8770/g1_29dof.urdf`, then `first sample received`.

Loco log contains `loco connected, mode damp`.

Sim log contains `simulator publishing on domain 1` and a real-time factor near `1.00`.

In the web app the sidebar lists `/robot/joint_states`, `/robot/base_pose`, `/tf`, the IMU topics, the velocity topic, `/scan`, and `/nav/state`. `/world/scene` appears only in a hallway world. The layout name is the imported `g1-layout`, and the 3D panel shows the robot. Service buttons are on the right. Teleop is under them.

## False alarms

**"Check that the WebSocket server at ws://localhost:8765 is reachable."** The bridge is not running. `task up` was stopped, or it has not logged the live-connection line yet. Reload the tab after that line appears.

**`PermissionError: Permission denied` while opening the MCAP, then the bridge exits.** Compose created `recordings/` as root because the directory did not exist yet. `task up` and the headless scripts create it as the host user, and take it back with `sudo` when it is not writable.

**Empty 3D, "image topic does not exist", diagnostics unavailable, while the topic list on the left is the `/robot/...` set.** The connection is fine. The open layout is an ANYbotics layout. Import `foxglove/g1-layout.json` and select it.

**The robot swings from the rope before Stand.** Motors are off. That is the start state, not a physics failure. It should settle rather than orbit. Stand, start the policy, lower, then release.

**The robot falls on release.** The policy was not balancing yet, or the rope was released while the feet were still off the ground. Follow the order in `README.md`. Watch pelvis height; feet take load near 0.8 m.

**`MESA: error: Failed to load driver: iris`.** The window still opens through the host X server. The NVIDIA GPU is not in this path. Real time on CPU is the expected performance.

**`selected interface "lo" is not multicast-capable`.** CycloneDDS falling back to unicast on loopback. Normal here.

**`[Reader] take sample error`.** A DDS read timed out before the first sample, or after the sim stopped. Harmless when `first sample received` follows, or during shutdown.

**`torch.jit.load` is deprecated.** Upstream's policy file is a TorchScript module. The warning is expected.

**Scan points a few centimetres from the robot.** The rangefinder rays pass through the arms when the gait swings a wrist up into the plane. The navigator ignores returns under 0.5 m. The recording keeps them.

**`/nav/state` says `blind` at once in the open world.** There are no walls to follow. Blind ends navigation and is not a fault; use a hallway world. Mid-hallway, `blind` means no scan arrived for half a second, which points at the sim.

**The robot turns away from the far wall at start.** The corridor axis is found, but the gait's yaw drift during stand-up left the robot facing the near end, and the direction was chosen once from what looked more open. Stop, turn it with Teleop, and start again. In the 20 m hallway both ends can be out of range at the start; the walker then keeps whichever way it was facing.

## Tests

Unit tests do not need a simulator: conversion, rope behavior, observation layout, services, the asset server, health classification, the MCAP check, the hallway model (compiled with MuJoCo but never stepped by the policy), and the navigator against synthetic scans. `tests/test_assets.py` skips the URDF joint-name assertion when the `unitree_rl_gym` submodule is absent.

`task e2e` is two checks. The walk must travel, stay above 0.5 m, never report `incident`, and write no event file. The walk sometimes falls; that failure is the regression, and the recording is what you open in Foxglove to see why. After a clean walk it releases the robot without the policy and requires health to reach `incident` and exactly one `events/*-incident-*.mcap` whose window ends 10 s after the fall. Do not point it at domain 1 while `task up` is up; the script defaults to domain 7.

`task e2e-hallway` passes when the gait stays up for the whole hallway, which today it does on a minority of runs; the failure mode is a fall a few metres in, logged as a health incident. It is kept as the acceptance check because the recording it leaves is the evidence for both the navigation and the gait. Do not loop it until it passes; note the pass rate with the change.

The fall rate moved without a code change. On 2026-09-30 `task e2e` passed four times in a row shortly after 02:00, then fell on every run after 02:30, including one with the hallway work stashed and the image rebuilt from the earlier commit. Physics cost is not it (a step with the fan is 0.13 ms against a 2 ms timestep) and the deps are lock-frozen. The host was idle by then with the CPU governor on `powersave` at 1.3 GHz, which is the first thing to compare on a machine where the walk suddenly falls: the two control loops sleep on wall time, and the upstream DDS publisher reads the MuJoCo data from its own thread. If the gait is stable when the machine is busy or the governor is `performance`, the fix is a timing one in `sim.py` or `loco.py`, not a policy one.

A shell with `UNITREE_DOMAIN_ID` exported will leak into local pytest through `Settings.from_env`. Unset it when a settings test fails only on that machine.

## Image and disk

`docker compose build` is the image `unitree-foxglove:local`. It is large because of the G1 meshes and the Python environment. Pruning images is safe; the next `task up` rebuilds. Do not prune while the user is mid-run unless they ask.

Recordings land in `recordings/` on the host. They are gitignored. `task up` creates that directory before Compose starts, so the daemon does not create a root-owned mount. The latest file can be tens or hundreds of megabytes after a long Teleop session.

## Changing the system

- A new Foxglove topic belongs in `convert.py`, `_log_sample`, `check_mcap`'s required set, the test fixture that writes a fake MCAP, and the layout if a panel should show it.
- A new service belongs in `build_services`, the layout, and `tests/test_bridge.py`.
- A new world is a `World` entry in `worlds.py` and a line in the `UNITREE_WORLD` table. `scripts/e2e_hallway.sh` needs a forward distance and a timeout for it.
- Navigation tuning is `NavParams` in `nav.py`; `tests/test_nav.py` runs the controller against a synthetic hallway with a yaw drift, which is the cheap place to see a change before Docker.
- A new robot needs a `RobotProfile` whose IDL family matches the scene (more than 20 actuators selects `hg` inside upstream's bridge) and whose joint names match that scene's sensor order.
- Do not vendor a second copy of `g1.yaml` numbers. Load the file.
- Do not edit `third_party/` to fix a behavior this process can wrap. `MotorFirmwareBridge` is the pattern: subclass, keep upstream's DDS setup, replace the part that is wrong.
