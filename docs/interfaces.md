# Interfaces

Names below are the contract. Change a topic or a service only together with `foxglove/g1-layout.json`, `foxglove/g1-remote-layout.json`, `foxglove/g1-mobile-layout.json`, `foxglove/mobile-field-operator.json`, `ws_client.py`, and the tests.

## Environment

Shared defaults are in `sim.env`. Compose passes the same names through so a one-off override (`FOXGLOVE_PORT=8766`) actually reaches the container.

| Variable | Default | Who reads it |
| --- | --- | --- |
| `UNITREE_ROBOT` | `g1` | all three (`go2` or `g1`) |
| `UNITREE_DOMAIN_ID` | `1` | all three |
| `UNITREE_INTERFACE` | `lo` | all three |
| `UNITREE_WORLD` | `open` | sim builds it, bridge draws it (`open`, `hallway10`, `hallway20`) |
| `UNITREE_HEADLESS` | `0` | sim |
| `UNITREE_DURATION` | `0` | sim, seconds, `0` means until stopped |
| `UNITREE_RECORD_DURATION` | `0` | bridge |
| `UNITREE_RECORD_PATH` | unset | bridge, exact output file |
| `RECORDINGS_DIR` | `/data/recordings` | bridge, directory of the timestamped MCAP |
| `UNITREE_RECORD_ROLL_S` | `0` in code, `60` from Compose | bridge, seconds per telemetry file; `0` or `UNITREE_RECORD_PATH` keeps one file |
| `FOXGLOVE_DEVICE_TOKEN` | unset | Foxlet and the bridge's Remote Access gateway, from `.env`, not from `sim.env`; the e2e scripts set it empty |
| `FOXGLOVE_HOST` | `127.0.0.1` | bridge |
| `FOXGLOVE_PORT` | `8765` | bridge |
| `ASSETS_PORT` | `8770` | bridge |
| `UNITREE_MUJOCO_ROOT` | `/app/third_party/unitree_mujoco` in the image | sim |
| `UNITREE_RL_GYM_ROOT` | `/app/third_party/unitree_rl_gym` in the image | loco, bridge |
| `DISPLAY` | host `DISPLAY` | sim, viewer only |

## DDS

Unitree topics, unchanged from upstream:

| Topic | Type | Direction |
| --- | --- | --- |
| `rt/lowstate` | `LowState_` (`hg` for G1, `go` for Go2) | sim publishes, bridge and loco subscribe |
| `rt/lowcmd` | `LowCmd_` | loco publishes, sim subscribes |
| `rt/sportmodestate` | `SportModeState_` | sim publishes, bridge subscribes |

Project topics, defined in `idl.py`:

| Topic | Type | Fields |
| --- | --- | --- |
| `loco/command` | `LocoCommand` | `mode` (`damp`, `stand`, `walk`), `vx`, `vy`, `wz`, `stamp_ns` |
| `sim/command` | `SimCommand` | `band_enable`, `band_length_delta` (metres, added to length), `reset` |
| `sensor/scan` | `LaserScan` | `stamp_ns`, `angle_min`, `angle_increment`, `range_max`, `ranges` (metres, `inf` for no return), `position`, `orientation` (w, x, y, z) of the sensor in `world` |

`sensor/scan` is published by the sim at 10 Hz from a 72-ray rangefinder fan on `torso_link`, 0.5 m above the link origin, 5° apart from -180°, range 10 m. The pose travels in the message so a consumer needs no TF. On a real G1 the same topic would carry a planar slice of the Livox point cloud.

G1 motor slots are the 35-wide `hg` array. The first 29 match `G1.joint_names` and the URDF's movable joints. Go2 uses 20 `go` slots and `GO2.joint_names`.

IMU quaternion on `rt/lowstate` is w, x, y, z. For the G1 it is the pelvis IMU (`imu_type: pelvis` in `g1.yaml`).

## Foxglove WebSocket

The SDK server (`foxglove.start_server`). In the app this is **Foxglove WebSocket**. With `FOXGLOVE_DEVICE_TOKEN` set, the same channels and services are also on the Remote Access gateway (`foxglove.start_gateway`) as device `g1-sim`. Its asset handler answers `package://<any>/<path>` from the robot description directory, which `foxglove/g1-remote-layout.json` uses for the URDF. See [Live connections](https://docs.foxglove.dev/docs/visualization/connecting/live) and [decisions.md](decisions.md#foxglove).

- URL: `ws://127.0.0.1:8765`
- Subprotocol: `foxglove.sdk.v1`
- Capabilities: client publish, services
- Client encoding: JSON

Channels the bridge logs, live and into the MCAP:

| Topic | Schema | Contents |
| --- | --- | --- |
| `/robot/joint_states` | `foxglove.JointStates` | 29 G1 joints, Unitree motor order |
| `/robot/base_pose` | `foxglove.PoseInFrame` | frame `world`; position from sport state, orientation from the IMU |
| `/tf` | `foxglove.FrameTransforms` | one transform, `world` → `pelvis` (or the profile's `base_frame`) |
| `/robot/base_velocity` | `foxglove.Vector3` | linear velocity |
| `/robot/imu_gyro` | `foxglove.Vector3` | rad/s |
| `/robot/imu_accel` | `foxglove.Vector3` | m/s² |
| `/cmd_vel` | JSON | Twist the app sent: `linear.x`, `linear.y`, `angular.z` |
| `/loco/mode` | JSON | `damp`, `stand`, `walk`, or `paused` when a service or a Twist requested it |
| `/health/state` | JSON | `state`, `reason`, `since_ns`; on change and at 1 Hz. Entering `incident` damps the motors once. See [health.md](health.md) |
| `/scan` | `foxglove.LaserScan` | frame `world`, `pose` from the DDS sample, `start_angle`, `end_angle`, `ranges`; 10 Hz |
| `/world/scene` | `foxglove.SceneUpdate` | one entity named after the world with a cube per wall, `frame_locked`, once a second; not logged in the open world |
| `/nav/state` | JSON | `state` (`idle`, `aligning`, `walking`, `arrived`, `blind`, `stopped`), `vx`, `wz`, `front_m`, `left_m`, `right_m` (`null` when unseen), `heading_deg`; 10 Hz while navigating, then once |

`/cmd_vel` is client-published. Anything else a layout advertises (for example `/move_base_simple/goal` from an ANYmal layout) is ignored.

Services, empty JSON request, response `{"ok": true, "detail": "..."}`. A refused request answers `ok: false` with the reason in `detail`:

| Service | Effect |
| --- | --- |
| `nav/start` | Walk the hallway to its end wall. Refused unless the policy is walking and not paused. `/nav/state` follows |
| `nav/stop` | Zero velocity, `/nav/state` logs `stopped`, Teleop drives again |
| `loco/stand` | mode `stand` |
| `loco/start` | mode `walk` at zero velocity until a Twist arrives |
| `loco/pause` | While walking: zero velocity, Twists dropped until resume. `/loco/mode` logs `paused`. `loco` still sees `walk` |
| `loco/resume` | Twists drive again. `/loco/mode` logs `walk` |
| `loco/damp` | mode `damp` |
| `sim/band_lower` | length += 0.1 m |
| `sim/band_raise` | length -= 0.1 m |
| `sim/band_release` | band off |
| `sim/band_attach` | band on, length unchanged |
| `sim/reset` | MuJoCo initial pose, band back on at length 0 |

Navigation ends on its own at `arrived` or `blind`, and is cancelled by a Twist on `/cmd_vel`, by `loco/stand`, `loco/start`, `loco/pause`, `loco/damp`, `sim/reset`, and by health entering `incident`. The last `/nav/state` says which.

Teleop in the layout publishes `/cmd_vel` at 5 Hz. Up/down are ±0.4 m/s forward. Left/right are ±0.6 rad/s yaw. The policy clamps to its training range: 0.8 m/s forward, 0.5 m/s sideways, 1.57 rad/s.

## Robot description HTTP

`http://127.0.0.1:8770/g1_29dof.urdf`, meshes under `http://127.0.0.1:8770/meshes/`. CORS allows any origin. The layout's URDF layer uses that URL, `controlMode: jointStates`, topic `/robot/joint_states`, mesh up axis `z_up`. Go2 has no URDF in this image, so the asset server stays off.

## Viewer keys

In the MuJoCo window, `7` shortens the rope by 0.1 m, `8` lengthens it, `9` toggles it. Same commands as the services.

## Exit codes

| Code | Tag | Meaning |
| --- | --- | --- |
| 64 | `SETUP-002`, `LOCO-001` | Unknown robot or world, or `loco` asked to run for a robot other than `g1` |
| 66 | `SETUP-001`, `LOCO-002` | Scene file missing, or policy / `g1.yaml` missing |
| 1 | `E2E-001` | WebSocket client failed |
| 2 | `E2E-002` | `ws_client --nav` ended in a navigator state other than `arrived` |

Processes call `os._exit` after a clean stop so CycloneDDS threads cannot keep the container up.

## Recording check

`check_mcap` requires messages on the joint, pose (including `/tf`), and IMU topics, and joint travel of at least 0.05 rad. With `--forward METRES` it also requires ground-plane displacement of at least that distance and minimum pelvis z of 0.5 m. Below that height the message is that the robot fell. With `--no-incident` it requires `/health/state` and fails if any message is `incident`. With `--nav-arrived` it requires `/scan` messages and a `/nav/state` timeline whose last state is `arrived`.

A recording in which health enters `incident` also gets `events/<run>-incident-<UTC>.mcap` beside it: one metadata record named `event`, plus two `/event` messages at the window bounds so Foxglove dates the file. A recovered `degraded` episode gets `events/<run>-degraded-<UTC>.mcap` with `kind` `degraded`, written 3 s after the last `degraded` sample. Keys and the Foxlet pattern: [health.md](health.md#incident-events).
