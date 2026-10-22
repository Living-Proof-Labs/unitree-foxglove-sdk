# Architecture

Three containers share one image and the host network. Host networking is required: CycloneDDS on `lo` does not cross a bridge network.

```
Foxglove web app
    |  WebSocket ws://127.0.0.1:8765  (subprotocol foxglove.sdk.v1)
    |  HTTP     http://127.0.0.1:8770 (URDF and meshes)
    v
bridge  ---- loco/command (DDS) ---->  loco
    |                                      |
    | sim/command (DDS)                    | rt/lowcmd
    v                                      v
                  sim
         rt/lowstate, rt/sportmodestate
              (read by bridge and loco)
         sensor/scan (read by bridge)
```

`bridge` is the only process the app talks to. It does not run physics or the policy. It turns panel actions into the two project DDS types and turns Unitree state into Foxglove messages. The hallway navigator also lives here: it reads `sensor/scan` and drives the same `LocoBackend` that Teleop does, so the sim, `loco`, and a future real robot see nothing but velocity commands.

`loco` is the stand-in for the robot's onboard locomotion service. It subscribes to `rt/lowstate` and `loco/command`, and publishes `rt/lowcmd`.

`sim` loads the MuJoCo scene, hosts Unitree's Python DDS bridge, applies the rope, and closes the motor PD loop inside each physics step.

## Code map

| Path | Owns |
| --- | --- |
| `src/unitree_foxglove/sim.py` | Scene, viewer, rope, physics step, in-step PD, `sim/command`, `sensor/scan` |
| `src/unitree_foxglove/worlds.py` | Hallway worlds and the lidar fan, added to the upstream scene through `MjSpec` |
| `src/unitree_foxglove/loco.py` | Policy load, damp / stand / walk, `loco/command`, `rt/lowcmd` |
| `src/unitree_foxglove/bridge.py` | Foxglove server, services, MCAP, `LocoBackend` seam, navigator wiring |
| `src/unitree_foxglove/nav.py` | `hallway_command` (scan in, velocity out) and `NavController` behind `nav/start` |
| `src/unitree_foxglove/assets.py` | HTTP server for the URDF directory |
| `src/unitree_foxglove/convert.py` | Unitree sample to Foxglove message |
| `src/unitree_foxglove/health.py` | Observe-only health classifier behind `/health/state` |
| `src/unitree_foxglove/incidents.py` | Metadata-only incident and degraded MCAPs under `recordings/events/` |
| `src/unitree_foxglove/robots.py` | `go2` and `g1` profiles: IDL family, scene, joint names, URDF |
| `src/unitree_foxglove/idl.py` | `LocoCommand`, `SimCommand`, and `LaserScan` |
| `src/unitree_foxglove/ws_client.py` | Headless client that drives the same services Teleop uses, with `--nav` for the hallway |
| `src/unitree_foxglove/check_mcap.py` | Recording acceptance check |
| `src/unitree_foxglove/stand.py` | Go2 stand-up trajectory only |
| `foxglove/g1-layout.json` | 3D URDF layer with `/scan` and `/world/scene`, plots, service buttons, Teleop, health and nav indicators |
| `foxglove/g1-remote-layout.json` | The same layout with the URDF at `package://g1_description/…`, served by the Remote Access gateway |
| `foxglove/g1-mobile-layout.json` | Phone layout: robot following the pelvis, health strip, stand-up buttons, Pause, Resume, Teleop |
| `foxglove/mobile-field-operator.json` | Field operator: health and nav color bars, stand-up buttons, Pause, Resume, Teleop. No 3D |
| `scripts/e2e.sh`, `scripts/e2e_hallway.sh` | Headless walk-and-fall, and headless hallway navigation |
| `compose.yaml`, `Taskfile.yml`, `Dockerfile` | Three services, one image |

`third_party/` is upstream and unmodified:

- `unitree_mujoco` — scenes and `simulate_python/unitree_sdk2py_bridge.py`
- `unitree_sdk2_python` — DDS types and the CycloneDDS client
- `unitree_rl_gym` — `deploy/pre_train/g1/motion.pt`, `deploy/deploy_real/configs/g1.yaml`, and `resources/robots/g1_description/` (URDF and meshes)

The image copies only those policy, config, licence, URDF, and mesh files from `unitree_rl_gym`. The rest of that submodule stays out via `.dockerignore`. Training code needs Isaac Gym and is not used.

## Control seam

`LocoBackend` in `bridge.py` is the place a real robot would plug in. `SimLocoBackend` publishes `loco/command`. `G1LocoClientBackend` documents the mapping onto `LocoClient` (`Damp`, `StandUp`, `Start`, `Move`) and is not implemented.

Foxglove stays the same in either case: services and `/cmd_vel` do not know which backend is behind them. Neither does `NavController`; it calls `move` on whatever backend it was given.

## Worlds and the scan

`worlds.py` opens the upstream scene with `mujoco.MjSpec`, adds the four wall boxes of the selected `World`, one site per lidar ray on `torso_link`, and one rangefinder sensor per site, then compiles. The upstream files are not changed and the open world adds only the fan. The rangefinders are appended after upstream's sensors, so the indices Unitree's DDS bridge reads from `sensordata` are the same as before, and a model compiled this way is bit-identical to the plain scene apart from the additions.

Every 0.1 s `sim.py` copies the fan into a `LaserScan` on `sensor/scan`, with the site position and the torso quaternion in the message. `bridge.py` logs it as `foxglove.LaserScan` in the `world` frame on `/scan` and hands it to the navigator. The hallway itself is drawn once a second on `/world/scene` from the same `World` the sim built, so the picture and the physics cannot disagree.

## Navigator

`hallway_command` in `nav.py` is a pure function of one scan and the previous decision. It finds the corridor axis as the dominant direction of the wall segments in the whole scan (a length-weighted doubled-angle histogram with hysteresis towards the previous axis), classifies the returns near the two lines parallel to that axis as side walls, takes the nearest remaining return inside a 1 m wide strip along the axis as the wall ahead, and picks the direction along the axis once, at start, towards the more open end. The command is a heading error (axis plus a centring bias) turned into yaw rate, and a forward speed that ramps down from 3.5 m to a stop at 1 m. Arrival is declared only when the wall ahead closed in slowly, which distinguishes a walk from a fall that sweeps the fan through the wall.

`NavController` runs it at 10 Hz in its own thread, smooths the yaw command, publishes `/nav/state`, refuses to start unless the policy is walking and not paused, and stops on `arrived`, `blind` (no scan for 0.5 s), an operator's `nav/stop`, a Twist, a mode change, or health entering `incident`. It never sets a loco mode; it only sends velocities.

## Modes

`loco` starts in `damp` (zero stiffness, light damping). `stand` ramps every joint to the policy default pose over two seconds. `walk` runs the policy at its control period. Entering `walk` or `stand` from `damp` ramps to the default pose first, so the policy is never asked to balance from a limp pose in one tick. Pause is not a `loco` mode: `SimLocoBackend` sends `walk` at zero velocity and drops Teleop until Resume. A velocity command older than half a second is treated as zero, which is what happens when the Teleop button is released during a walk.

The policy commands the 12 leg joints. Waist and arms hold the targets in `g1.yaml`. Drift at zero command is expected.

## Rope and stand-up order

The rope is on at start, length zero, anchored above the torso. The robot hangs limp until Stand. The policy must already be balancing before the feet take weight. A fixed pose falls as the rope goes slack. The order is stand, start, lower (about seven steps of 0.1 m), release, then drive. Reset puts the robot back on the rope at the initial pose. Damp first, so the controller is not fighting the reset.
