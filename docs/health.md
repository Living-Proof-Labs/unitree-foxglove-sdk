# Health state and incident events

Health answers one question for whoever is watching: is the robot fine. It observes. The one command it sends is damp, and only on the transition into `incident`, so a fall stops the policy instead of leaving it kicking. No layout button reacts to it. A later `incident` sample does not send damp again. Teleop does not leave damp; **Start policy** or **Stand** does.

## Where it runs

In the bridge process, next to the recorder. On a real robot that process runs on the robot, so the state exists without anyone watching, and it is in every recording. The layout only displays it.

A Foxglove User Script could compute the same thing inside a layout. That is handy while testing a threshold, but it exists only while someone has the layout open and it is not in the MCAP, so it is not the source of truth.

## States

| State | Meaning |
| --- | --- |
| `nominal` | Upright, standing or walking under the policy |
| `degraded` | Still upright, but tilt or height is outside the normal band |
| `incident` | Fallen: pelvis too low or tilted too far, held long enough to rule out a stumble |
| `inactive` | Not judged: motors damped, or still hanging on the rope |
| `offline` | No robot state arriving |

Thresholds and hold times are constants in `src/unitree_foxglove/health.py`. The classifier is pure functions over pelvis height, IMU gravity direction, loco mode, and rope state, so it is tested without a simulator.

For the G1: `incident` below 0.45 m or past 60 degrees of tilt, held 0.5 s; `degraded` below 0.65 m or past 25 degrees. During the hold the state reads `degraded`. `offline` after 1 s with no `rt/lowstate`. Pause counts as `nominal`: the policy is still balancing.

The loco mode and rope state are what the bridge last requested, not what `loco` or the simulator report. A bridge restarted mid-run assumes damp and the rope on until the next button.

## Topic

`/health/state`, JSON, published on every change and at 1 Hz:

```json
{"state": "incident", "reason": "pelvis 0.21 m below 0.45 m", "since_ns": 1790640000000000000}
```

The layout shows it in a State Transitions panel, so a run reads as a timeline of states, and an Indicator that turns red on `incident`.

## Incident events

A transition into `incident` opens a fixed window: 30 s before health entered `incident` (or the start of this recording, if that is later) through 10 s after. The file is written once, at the end of that window. It does not wait for the robot to get back up. Another fall writes another file. Stopping the bridge early closes an open window at that moment, so `end` is sooner than 10 s.

The file is `recordings/events/<run>-incident-<UTC>.mcap`. It holds a metadata record named `event`, and the same fields again as two `/event` messages logged at `start` and `end`. Foxglove dates a recording from its messages; without those two, the file is dated 1970-01-01 and cannot overlap the telemetry.

| Key | Value |
| --- | --- |
| `kind` | `incident` or `degraded` |
| `start`, `end` | The window, ISO-8601 UTC |
| `at` | When health entered `incident` |
| `reason` | The health reason, for example `pelvis 0.21 m below 0.45 m` |
| `source_recording` | The telemetry MCAP this window belongs to |
| `producer` | `unitree-foxglove` |

The telemetry MCAP is the data. The event file is a time span small enough to upload on every incident. The name carries the kind (`-incident-` or `-degraded-`) so a directory listing shows which is which. Foxlet does not care about that: it imports `**/events/**/*.mcap`, including an `events/` directory under an e2e run.

## Degraded events

A run of `degraded` that recovers is one file, `recordings/events/<run>-degraded-<UTC>.mcap`, with the same metadata record. `kind` is `degraded`. `at` is the first `degraded` sample. `reason` is the latest reason in the episode. `start` is 10 s before `at` (or the start of the recording). `end` is 3 s after health last left `degraded`.

Samples that return to `degraded` inside those 3 s extend the same episode. A gap longer than 3 s writes the file and starts another if `degraded` comes back. The file is written when the quiet stretch elapses. Stopping the bridge sooner writes it then, with `end` at that moment.

A `degraded` stretch that becomes `incident` without recovering does not get its own file. The fall is the incident, and that window already covers the lead-in. A bump that recovers and then, after the quiet stretch has started, becomes a fall still writes the degraded file: that recovery happened.

## Getting an event into Foxglove

This repository does not call the Foxglove API. The robot may have no link when it falls, and it should not hold an events key.

Foxlet is the `foxlet` service in `compose.yaml`, enabled by `task up` when `.env` contains `FOXGLOVE_DEVICE_TOKEN`. Create the token in your Foxglove organization for the device that should receive the recordings. `.env.example` shows the variable. `.env` is gitignored. A token that is not already tied to a device needs `FOXGLOVE_DEVICE_NAME` (compose uses `g1-sim`); Foxlet creates the device on first start. `WATCH_AUTO_IMPORT_PATTERN=**/events/**/*.mcap` uploads incident and degraded files as soon as they are closed. Telemetry MCAPs are watched and listed, and are not uploaded until something asks for them.

The bridge closes a telemetry file every `UNITREE_RECORD_ROLL_S` (60 s on `task up`, off for `task e2e`). A closed file is what Foxlet can hand to an import. An event window then pulls one or two of those files instead of a whole session.

A separate project, `foxglove-event-trigger`, is the consumer. It is not in this repo. It is woken by the `recording.imported` webhook for every imported recording. If `recording.path` does not match `events/*-incident-*.mcap` or `events/*-degraded-*.mcap` it stops. If it matches, it reads the `event` metadata from `GET /v1/recordings/{id}` (the file body is not downloaded) and `POST /v1/events` with that device, `start`, `end`, and the metadata strings, including `kind`. The key lives in the pipeline.

Creating that event does not upload the telemetry. An event is a device and a time span. Visualize opens recordings that are already imported and overlap the span. A file Foxlet has only noticed stays on the robot until import is requested. After creating the event, the trigger lists coverage for the device and window with `includeEdgeRecordings=true` and `POST /v1/recordings/{id}/import` for each overlapping file. Those files upload whole, which is why the telemetry rolls. Logging stays dumb, and only the files that cover the episode move.
