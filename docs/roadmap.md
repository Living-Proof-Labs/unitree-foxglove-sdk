# Roadmap

The working benchmark is a G1 that stands from the rope, walks under the pretrained policy, and is driven from Foxglove, with the run recorded as one MCAP. This file is the order of what comes next. It is not a commitment to build it all in one change.

Each new locomotion behavior is a `loco/*` service and a button in `foxglove/g1-layout.json`. The WebSocket, the Teleop topic, and the DDS seam stay as they are. That is the extensibility the remote-control panel is there to show.

## What the buttons do today

- **Stand** ramps to the walking default pose. It is the right move while the robot is hanging on the rope. From a fallen pose it does not get up, and the walking gains fight the ground.
- **Start** runs the walking policy. It keeps balancing and tracking velocity.
- **Damp** sets stiffness to zero. The robot collapses. That is motors-off, not a stop to request with a person nearby.

## Tried and removed: Stop as a crouch

A crouch held with soft gains and the policy off fell over in the first manual test: pelvis height stepped down to about 0.1 m. Nothing was balancing. A still pose the robot can come back from needs a balance controller running, and the pretrained walking policy is the only one available. See [decisions.md](decisions.md#locomotion).

## Next, in order

1. **Pause and Resume.** Implemented. Pause keeps the walking policy balancing at zero velocity and ignores Teleop. Resume lets Teleop drive again. The robot still shifts its feet while paused; that is what this policy does at zero speed. A confused policy will keep stepping while paused. The calm pose is quiet stand, below, not a change to Pause. How to check it: [operations.md](operations.md#validate-pause-and-resume).
2. **Health state.** Implemented. `nominal`, `degraded`, `incident`, `inactive`, `offline`, computed next to the recorder and shown in the layout. Entering `incident` damps the motors so a fall does not keep the policy kicking. No other health state sends a command. See [health.md](health.md); how to check it: [operations.md](operations.md#validate-the-health-state).
3. **Incident events.** The event MCAP is written. `foxglove-event-trigger`, the project that turns that file into a Foxglove event and then asks for the overlapping logs, is not started. See [health.md](health.md#incident-events); how to check the file: [operations.md](operations.md#validate-incident-events).
4. **Get up.** A recovery from lying on the floor to the standing pose, then idle. Separate from Stand. The pretrained walk policy does not include it, so it is its own controller. With health in place, an incident is what would offer it.
5. **Quiet stand.** A controller that holds still without stepping. It would make Pause look calmer. Also its own controller.

## The hallway

Done:

- Opt-in hallway worlds, `hallway10` and `hallway20`, closed at both ends. The open floor stays the default so `task up` and `task e2e` are unchanged. `task hallway WORLD=...`.
- A lidar fan on the torso, in the 3D panel on `/scan`, and the walls on `/world/scene`.
- A navigator for space the robot has no map of: `nav/start` walks the centre of the hallway to the wall that ends it and stops 1 m short. `task e2e-hallway` is its check. How to try it: [operations.md](operations.md#validate-the-hallway-walk); why it is shaped the way it is: [decisions.md](decisions.md#navigation).

Not started:

- A live camera in the image panel, and the split into `*-telemetry.mcap` (decimated state, the scan, the scene, health, no images, no point clouds, no username or hostname) and `*-sensors.mcap` (camera and point cloud, not uploaded). Today there is one MCAP and it already holds no image or personal data.
- A goal from the Teleop panel or a map. That is localisation in a known space, the later step.
- A turn at a corner, or a choice at a junction. The corridor follower assumes one axis; a bend would need it to re-pick the axis when the wall ahead closes and a side opens.

## Then a real G1 EDU

The navigator, the services, and the layout do not depend on the simulator. What a real robot needs, in order:

1. `G1LocoClientBackend` implemented on `LocoClient` (`Damp`, `StandUp`, `Start`, `Move`) on domain 0 and the robot's interface. The `bridge` container would run on a laptop on the robot's network.
2. A `sensor/scan` publisher that takes one horizontal slice of the Livox Mid-360 cloud at torso height and sends it as this repo's `LaserScan` with the sensor pose. Everything downstream of that topic is already written.
3. Retune `NavParams` on the real gait, which tracks its velocity command; the sim's overshoot pushed `cruise_speed` down and `stop_distance_m` up.
4. Health thresholds checked against the real pelvis IMU, and the incident MCAP kept on the robot's disk for Foxlet.

The rope, the band services, and `sim/reset` do not carry over. The operator stands the robot with Unitree's app and hands it to `loco/start`.
