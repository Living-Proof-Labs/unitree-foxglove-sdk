# Overview

A Unitree G1 EDU (29 degrees of freedom) runs in MuJoCo inside Docker. Foxglove is the live view, the remote control, and the recorder. The simulator does not include the locomotion service a real G1 would run onboard, so a separate process runs Unitree's pretrained walking policy and speaks the same low-level DDS topics the simulator does.

The benchmark that defines "working":

- The robot starts hanging from a virtual rope with the motors off.
- Stand, then start the policy, then lower and release the rope.
- It balances, and Teleop walks it around.
- The same channels are on `ws://localhost:8765` and in `recordings/<robot>-<UTC>.mcap`.
- `task e2e` repeats that sequence headlessly and requires forward travel without a fall, then drops the robot without the policy and requires an incident event file.

## Features

- **Simulator.** MuJoCo scene for the G1 29-dof model, Unitree DDS state out and motor commands in, optional viewer. Go2 loads as a second profile without the walking policy.
- **Rope.** A virtual band holds the humanoid up at start, the way Unitree's own humanoid sim does. Length and release are controllable from Foxglove and from viewer keys.
- **Locomotion stand-in.** Damp, stand, and walk. Walk is the pretrained `motion.pt` policy with the gains and observation layout from Unitree's real-robot deploy config.
- **Foxglove bridge.** WebSocket server with client publish and services, plus an MCAP of the same data. Teleop arrives as Twist on `/cmd_vel`. Stand, start, damp, and the rope are services.
- **Robot model in the 3D panel.** The bridge serves the URDF and meshes. The layout poses the robot from joint states and a `world → pelvis` transform.
- **Headless check.** A small WebSocket client performs the stand-up and a short drive. A recording check requires joint motion, ground-plane travel, and a minimum pelvis height.
- **Hallway worlds and a lidar.** `UNITREE_WORLD=hallway10|hallway20` closes the robot into a 2 m wide hallway drawn in the 3D panel, and a 72-ray planar scan from the torso is on `/scan`. Both are added to the upstream scene at load time.
- **Hallway navigator.** `nav/start` walks the centre of a hallway it has no map of and stops 1 m before the end wall. `/nav/state` shows what it sees. `task e2e-hallway` checks it headlessly.

## Boundaries

- Docker only. No native install path.
- DDS domain 1 on the host loopback, so this stack does not talk to a real robot on domain 0.
- The real-robot backend is a named stub (`G1LocoClientBackend`). It raises `NotImplementedError`. Do not implement it unless the user asks to leave the simulator. What that step would take is in [roadmap.md](roadmap.md#then-a-real-g1-edu).
- Navigation is a corridor follower. No map, no localisation, no goal input. A turn or a junction is not handled.
- The walking policy is G1-only. Other robot names make `loco` exit.
- Foxglove layouts for other robots may subscribe to cameras, diagnostics, and navigation topics this bridge does not publish. The layout for this connection is `foxglove/g1-layout.json`.
- No git remote. Do not push.
