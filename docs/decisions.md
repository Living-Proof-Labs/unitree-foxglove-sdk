# Decisions

Each item is settled. The short reason is the failure mode, not a lab notebook. Change one only when the user is changing that behavior.

## Simulator and transport

**Python MuJoCo from `unitree_mujoco`, not the C++ simulator and not NVIDIA Isaac.** The Python scene already speaks Unitree DDS. Isaac Gym is a training dependency of `unitree_rl_gym` and is not required to run the pretrained policy.

**Docker with host networking, user `${UID}:${GID}`.** The viewer needs the host X socket. Recordings are a bind mount. A Compose bridge network drops CycloneDDS on `lo`. `UID` is readonly in bash, so Task and `scripts/e2e.sh` pass it in the environment.

**DDS domain 1, interface `lo`.** Domain 0 is what a real Unitree robot uses. This stack stays off it.

**Python 3.10.** `unitree_sdk2py` pins `cyclonedds==0.10.2`, whose newest wheel is CPython 3.10. A newer interpreter means building CycloneDDS from source.

**CPU torch, explicit uv index.** The policy is a small MLP. The default PyPI torch wheel pulls a large CUDA build. `pyproject.toml` points torch at the PyTorch CPU index and marks that index explicit so nothing else resolves from it.

## Physics

**PD torque is computed inside the physics step from current sensors.** Upstream's bridge converts each `rt/lowcmd` into one torque on arrival, from sensors that are already a step old, and holds it. With the G1 gains that torque diverges, MuJoCo auto-resets, and the base appears to snap back to the origin or fall through the floor. `MotorFirmwareBridge` stores targets and gains, then `mj_step1` (sensors current) → `ctrl` → `mj_step2` each step. That matches a motor driver closing the loop on fresh measurements while the policy sends targets at 50 Hz.

**Physics timestep stays the model's (2 ms). DDS state is published at 5 ms.** Publishing at 500 Hz from Python starves the step thread and the real-time factor drops below 1. The upstream bridge reads its publish interval from `model.opt.timestep` at construction, so the sim sets that to 5 ms, builds the bridge, and restores 2 ms. A healthy run prints `real-time factor 1.00`.

**The rope is stiff, slack when short, and damped in every direction.** Upstream's 200 N/m spring stretches about the robot's weight and never hands the load to the feet. 2000 N/m with zero force when `distance <= length` behaves like a gantry: each 0.1 m of length lowers the pelvis about 0.1 m, and after about seven steps the rope is slack. Damping only along the rope leaves an undamped pendulum, which looks like the robot flying. Damping the full velocity while taut lets a limp robot settle. The force is applied to `torso_link`. The pose Foxglove shows is the pelvis IMU.

**Worlds and the lidar are added to the compiled model with `MjSpec`, not by editing or copying scene XML.** `third_party/` stays pinned, and there is one scene file to keep in step with upstream. The rangefinders are appended after upstream's sensors so `sensordata` indices the Unitree bridge reads do not move. MuJoCo 3.14 requires `sensor.intprm[0] = 1` on a rangefinder; without it the compile fails with "data spec must be positive".

**The lidar is a rangefinder fan on `torso_link`, 0.5 m up, 72 rays, 10 m.** A rangefinder ray ignores the body it is mounted on (torso and head) but not the arms, and this gait swings the wrists up through the plane. Returns under 0.5 m are blanked in the navigator, not in the sim, so the recording still shows what the sensor saw. A camera and a full point cloud are not part of the telemetry channel and are not in this fan.

## Locomotion

**The policy is Unitree's `motion.pt` plus `deploy/deploy_real/configs/g1.yaml`, not a reimplementation and not a retrain.** Observation size is 47, actions are 12, control period is 0.02 s. `deploy_real.py` is not imported; it depends on `legged_gym` and Isaac Gym. The numbers live in the YAML. Do not copy them into code.

**Walk is not a high-level sport API.** The simulator has no `LocoClient`. Low-level `rt/lowcmd` is the interface both the policy and a future real-robot backend sit behind. See `LocoBackend`.

**Pausing keeps the policy running.** A passive pose without a balance controller falls; a soft-gain crouch did, in the first manual test. Pause is the walking policy at zero velocity with Teleop ignored, so Resume is instant and the robot never has to recover from a fall. It still steps in place. If the policy is confused by bad sensing it will keep trying to step while paused; a calmer pose is the quiet-stand controller, on the roadmap, not a change to Pause.

**The pretrained gait, coupled to this simulator, walks about twice its commanded speed, drifts in yaw at zero command, and falls on some runs.** It does so with the committed code before any world or navigator existed, with the deps frozen by the lock file, with DDS at a steady 5 ms, and with the CPU idle. It is not caused by the walls, the fan, or the navigator; the compiled hallway model is bit-identical to the plain scene apart from the additions, and open-loop trajectories match. Tune around it (slower cruise, earlier stop, yaw smoothing) rather than chasing it in this repo. Get-up and quiet stand on the roadmap are the answers to what happens after a fall.

## Navigation

**The navigator is a corridor follower for space it has no map of.** The request was an algorithm for an unknown space, not a path through a known one. Each tick uses one scan and the previous decision. There is no occupancy grid, no localisation, and no waypoint. A goal from a panel or a map comes after this, as its own controller.

**The corridor axis comes from the whole scan, not from the two rays beside the robot.** The policy can leave the robot facing a side wall, and the gait turns it while walking. Left and right rays then read the end wall or nothing. A length-weighted histogram of wall-segment directions finds the axis at any heading; hysteresis towards the previous axis keeps the short end wall from out-voting the long side walls up close.

**Which end to walk to is decided once.** At start the more open end along the axis is chosen. After that the controller keeps the direction closest to the previous one through every turn. Re-deciding each tick made a robot that had drifted to face backwards walk back to the near wall and declare that the end.

**Turn in place before walking when the heading error is above 30°.** The gait tolerates less yaw while walking than standing, and walking sideways into a wall 1 m away is the failure the alignment step prevents.

**Arrival needs a slow approach.** The clearance ahead must have been within 1.8 m one second before it read 1 m. A fall pitches the fan into the wall or the floor in a fraction of a second and otherwise reads as a perfect arrival.

**Navigation stops when anything else asks for control.** A Twist, a loco mode change, `sim/reset`, or health entering `incident` stops it and logs the reason on `/nav/state`. An incident then damps the motors, so the policy is not left walking at zero velocity on the floor.

**The scan pose rides in the message, not in `/tf`.** The navigator and Foxglove need the pose the scan was taken at. `/tf` carries only `world → pelvis` for the URDF, and the fan is on the torso. Sending the site pose with the ranges avoids a second transform and a timing question, and is the same shape a real lidar driver would publish.

## Health

**Health commands exactly once: damp on the way into `incident`.** It runs next to the recorder so it exists on the robot and in every MCAP. A layout User Script is a view for testing thresholds, not the source of truth. `degraded`, `nominal`, and a repeated `incident` send nothing. Damp is the motors-off pose, which is what you want under a robot that is already down; Pause would keep the policy stepping. After damp the next sample reads `inactive` (`motors damped`), because a damped robot is not judged. The incident state and the event file already recorded the fall. A Twist does not leave damp, so a stick still held down cannot restart the thrash. **Start policy** or **Stand** is the way back.

**A health episode is a tiny recording, not an API call from the robot.** The file under `recordings/events/` carries the time window in an MCAP metadata record and in two `/event` messages at the bounds. Without the messages Foxglove dates the file 1970-01-01. The filename contains `incident` or `degraded`; the metadata `kind` is the same word. Foxlet uploads `**/events/**/*.mcap` and leaves the telemetry listed until `foxglove-event-trigger`, which holds the key, creates the Foxglove event and asks for the overlapping recordings. A recovered `degraded` run is one file, closed after 3 s without another `degraded` sample. A `degraded` run that becomes a fall is not a second file.

## Foxglove

**Foxglove SDK server, not `foxglove_bridge` and not a ROS graph.** There is no ROS in this stack. The server must advertise the subprotocol `foxglove.sdk.v1`. A client that omits it gets HTTP 400. The app's **Foxglove WebSocket** entry is this server. **Remote Access** is the same SDK via `start_gateway` and a device token, for a viewer who cannot reach the host. The bridge starts a gateway beside the server only when `FOXGLOVE_DEVICE_TOKEN` is set, for a phone or another machine. It shares the services and Teleop handling, and serves the URDF and meshes to `package://` URIs through its asset handler, because a remote browser cannot reach `http://localhost:8770`. The e2e scripts clear the token. Do not replace `start_server` with the gateway: localhost wants every message, and the e2e client speaks the WebSocket. The gateway and Foxlet share one device token and one device without conflict. Foxglove's comparison is [Live connections](https://docs.foxglove.dev/docs/visualization/connecting/live). The deprecated server is a hand-rolled `foxglove.websocket.v1` implementation, not either SDK entry.

**The web app is the supported viewer.** `https://app.foxglove.dev` can open `ws://localhost:8765` because browsers treat localhost as a secure origin. `task foxglove` opens that deep link. It does not switch layouts. The user must import `foxglove/g1-layout.json` once.

**The G1 mesh is an HTTP URDF layer, not a TF tree of every link.** The bridge serves `g1_29dof.urdf` and `meshes/` on port 8770 with `Access-Control-Allow-Origin: *`, which the page at `app.foxglove.dev` needs to fetch localhost. Mesh paths in the URDF are relative, so one directory is enough. `/tf` carries only `world → pelvis`. Joint motion comes from `/robot/joint_states` with control mode `jointStates`. The URDF's movable joint names match `G1.joint_names` in order. A partial joint-state message does not merge with the previous one.

**Layouts for other robots do not match this connection.** They stay blank or show "topic does not exist" while the WebSocket is healthy. The topic list in the sidebar is the connection check.

**Quaternion order differs.** Unitree IMU quaternions are w, x, y, z. Foxglove orientation is x, y, z, w. `convert.py` swaps them. Do not "fix" one side to match the other.

## Custom DDS types

**`LocoCommand` and `SimCommand` are CycloneDDS IDL structs in `idl.py`.** No `from __future__ import annotations` in that module. CycloneDDS reads the real annotation objects (`float64`, `int64` from `cyclonedds.idl.types`). Postponed annotations are strings and fail type resolution.

**`rt/lowcmd` for the G1 is the `hg` message.** `mode_pr` is 0, `mode_machine` is copied from the latest state, each motor `mode` is 1, and the CRC is `CRC().Crc(cmd)`. Skipping the CRC or the mode fields makes the simulator ignore the command.

## Checks

**`task e2e` is the regression for "it can walk", then a scripted fall.** It uses domain 7, Foxglove port 8766, and asset port 8771 so it can run beside `task up`. The walk requires at least 0.3 m of ground-plane travel and pelvis height staying at or above 0.5 m, and no incident event file. The fall requires health to reach `incident` and one event MCAP whose window ends 10 s after the fall. Joint travel below 0.05 rad fails even without the walk check. A recording with no `/tf` fails, because `/tf` is part of the pose channels.

**`task e2e-hallway` is the regression for "it can find the end of a hallway".** Same ports and domain as `task e2e`. It requires the last `/nav/state` to be `arrived`, `/scan` in the recording, at least 6 m of travel in the 10 m hallway (15 m in the 20 m one), no fall, no incident. It is not repeated on failure; a fall is reported as `stopped` with the health reason, and the MCAP under `recordings/e2e-hallway/` is the evidence.

**`LaserScan` on DDS is a project IDL struct with a `sequence[float32]`.** Same module and same rules as `LocoCommand`. Unitree's own point cloud types are not used, because a planar scan is what the navigator reads on either robot.

## Repository

**Submodules are pinned and unmodified.** Divergence belongs in `src/unitree_foxglove/`.

**Bags are not source.** `*.mcap` is gitignored. Review them in Foxglove.

**`foxglove` is a third-party import, not the layout directory.** Ruff would treat `foxglove/` as the first-party package. `known-third-party = ["foxglove"]` keeps the SDK import sorted correctly.
