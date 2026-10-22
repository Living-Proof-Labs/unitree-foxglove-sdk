"""One Foxglove connection for watching and driving the robot.

The bridge subscribes to the simulator's DDS state, writes an MCAP file, and
serves a Foxglove WebSocket on port 8765. The same server accepts input from
the app: the Teleop panel publishes ``geometry_msgs/Twist`` on ``/cmd_vel`` and
the Service Call panel invokes ``loco/*`` and ``sim/*`` services. Those become
DDS requests to the ``loco`` container (``loco/command``) and the simulator
(``sim/command``).

Everything the app sends is also logged, so a recording shows what was asked of
the robot next to what it did.

The lidar scan (``sensor/scan``) is relayed as ``/scan`` and feeds the hallway
navigator, which the ``nav/*`` services start and stop. When ``UNITREE_WORLD``
names a hallway, its walls are published on ``/world/scene`` so the 3D panel
shows the same world the simulator loaded.
"""

from __future__ import annotations

import json
import os
import signal
import sys
import threading
import time
from datetime import datetime, timezone
from http.server import ThreadingHTTPServer
from pathlib import Path
from typing import Protocol

import foxglove
from foxglove import Schema
from foxglove.remote_access import Capability as RemoteCapability
from foxglove.remote_access import (
    ChannelDescriptor,
    RemoteAccessConnectionStatus,
    RemoteAccessGateway,
    RemoteAccessListener,
)
from foxglove.remote_access import Client as RemoteClient
from foxglove.websocket import (
    Capability,
    Client,
    ClientChannel,
    MessageSchema,
    Service,
    ServiceRequest,
    ServiceSchema,
)
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

from unitree_foxglove.assets import serve_directory
from unitree_foxglove.convert import (
    TOPIC_BASE_POSE,
    TOPIC_BASE_VELOCITY,
    TOPIC_CMD_VEL,
    TOPIC_IMU_ACCEL,
    TOPIC_IMU_GYRO,
    TOPIC_JOINT_STATES,
    TOPIC_LOCO_MODE,
    TOPIC_SCAN,
    TOPIC_TF,
    TOPIC_WORLD_SCENE,
    base_pose_from_state,
    base_transform_from_state,
    base_velocity_from_sport,
    imu_from_lowstate,
    joint_states_from_lowstate,
    laser_scan_from_dds,
    scene_update_from_world,
    timestamp_from_ns,
    twist_from_json,
)
from unitree_foxglove.health import (
    INCIDENT,
    THRESHOLDS,
    TOPIC_HEALTH_STATE,
    HealthMonitor,
    HealthState,
    Observation,
    tilt_from_quaternion,
)
from unitree_foxglove.idl import (
    MODE_DAMP,
    MODE_STAND,
    MODE_WALK,
    TOPIC_LOCO_COMMAND,
    TOPIC_SIM_COMMAND,
    LaserScan,
    LocoCommand,
    SimCommand,
)
from unitree_foxglove.idl import TOPIC_SCAN as DDS_TOPIC_SCAN
from unitree_foxglove.incidents import IncidentRecorder
from unitree_foxglove.nav import TOPIC_NAV_STATE, NavController
from unitree_foxglove.robots import DEFAULT_ROBOT, RobotProfile, profile_for
from unitree_foxglove.sim import BAND_STEP_M
from unitree_foxglove.worlds import DEFAULT_WORLD, World, world_for

_READ_TIMEOUT_S = 0.5
SCENE_REPUBLISH_S = 1.0
"""How often the static walls are logged, so a seek in playback still finds them."""
DEFAULT_ASSETS_PORT = 8770
"""HTTP port for the URDF and meshes; the layout's URDF layer points here."""
PACKAGE_SCHEME = "package://"
LOG_MODE_PAUSED = "paused"
"""``/loco/mode`` label for a pause. Not a DDS mode: ``loco`` is still walking."""

_EMPTY_SCHEMA = MessageSchema(
    encoding="json",
    schema=Schema(
        name="Empty", encoding="jsonschema", data=json.dumps({"type": "object"}).encode()
    ),
)
_ACK_SCHEMA = MessageSchema(
    encoding="json",
    schema=Schema(
        name="Ack",
        encoding="jsonschema",
        data=json.dumps(
            {
                "type": "object",
                "properties": {"ok": {"type": "boolean"}, "detail": {"type": "string"}},
            }
        ).encode(),
    ),
)


class LocoBackend(Protocol):
    """What the app can ask the locomotion controller to do.

    Names follow ``unitree_sdk2py.g1.loco.g1_loco_client.LocoClient`` so a real
    robot backend is a thin mapping: ``Damp`` / ``StandUp`` / ``Start`` /
    ``Move(vx, vy, vyaw)``.
    """

    mode: str

    def damp(self) -> None: ...

    def stand(self) -> None: ...

    def start(self) -> None: ...

    def move(self, vx: float, vy: float, wz: float) -> None: ...

    def pause(self) -> None: ...

    def resume(self) -> None: ...

    @property
    def paused(self) -> bool: ...


class SimLocoBackend:
    """Publishes ``loco/command`` for the ``unitree-loco`` process.

    Pause is held here, not in ``loco``: the walking policy keeps balancing at
    zero velocity and Teleop is dropped until Resume, so ``loco`` only ever sees
    ``walk``. A pose without the policy running falls over.
    """

    def __init__(self) -> None:
        self._publisher = ChannelPublisher(TOPIC_LOCO_COMMAND, LocoCommand)
        self._publisher.Init()
        self.mode = MODE_DAMP
        self._paused = False

    @property
    def paused(self) -> bool:
        return self._paused

    def damp(self) -> None:
        self._paused = False
        self._send(MODE_DAMP, 0.0, 0.0, 0.0)

    def stand(self) -> None:
        self._paused = False
        self._send(MODE_STAND, 0.0, 0.0, 0.0)

    def start(self) -> None:
        self._paused = False
        self._send(MODE_WALK, 0.0, 0.0, 0.0)

    def pause(self) -> None:
        if self.mode != MODE_WALK:
            return
        self._paused = True
        self._send(MODE_WALK, 0.0, 0.0, 0.0)

    def resume(self) -> None:
        self._paused = False

    def move(self, vx: float, vy: float, wz: float) -> None:
        # Damp is motors off until an explicit stand or start. A Twist, including
        # one already in flight from the navigator, must not turn the policy
        # back on after a fall.
        if self._paused or self.mode == MODE_DAMP:
            return
        self._send(MODE_WALK, vx, vy, wz)

    def _send(self, mode: str, vx: float, vy: float, wz: float) -> None:
        self.mode = mode
        self._publisher.Write(LocoCommand(mode, float(vx), float(vy), float(wz), time.time_ns()))


class G1LocoClientBackend:
    """Placeholder for the real G1.

    Would wrap ``LocoClient`` from ``unitree_sdk2py.g1.loco.g1_loco_client``:
    ``damp`` -> ``Damp()``, ``stand`` -> ``StandUp()`` then ``BalanceStand(0)``,
    ``start`` -> ``Start()``, ``move`` -> ``Move(vx, vy, wz)``. Requires the robot's
    network interface and DDS domain 0. Not implemented in this trial.
    """

    def __init__(self) -> None:
        raise NotImplementedError("Real G1 backend is not part of this trial.")


class SimControl:
    """Publishes ``sim/command`` for the elastic band and reset."""

    def __init__(self) -> None:
        self._publisher = ChannelPublisher(TOPIC_SIM_COMMAND, SimCommand)
        self._publisher.Init()
        self.band_enabled = True

    def band_release(self) -> None:
        self.band_enabled = False
        self._publisher.Write(SimCommand(False, 0.0, False))

    def band_attach(self) -> None:
        self.band_enabled = True
        self._publisher.Write(SimCommand(True, 0.0, False))

    def band_lower(self) -> None:
        self._publisher.Write(SimCommand(self.band_enabled, BAND_STEP_M, False))

    def band_raise(self) -> None:
        self._publisher.Write(SimCommand(self.band_enabled, -BAND_STEP_M, False))

    def reset(self) -> None:
        self.band_enabled = True  # the simulator re-hangs the robot on reset
        self._publisher.Write(SimCommand(True, 0.0, True))


class AppListener:
    """Receives Teleop messages and remembers which client channel is ``/cmd_vel``.

    A Teleop message while the navigator is walking cancels it: the operator's
    hand on the stick wins over the autonomy.
    """

    def __init__(
        self, loco: LocoBackend, log_lock: threading.Lock, nav: NavController | None = None
    ) -> None:
        self._loco = loco
        self._topics: dict[tuple[int, int], str] = {}
        self._log_lock = log_lock
        self._nav = nav

    def on_client_advertise(self, client: Client, channel: ClientChannel) -> None:
        self._topics[(client.id, channel.id)] = channel.topic
        print(f"client {client.id} advertised {channel.topic} ({channel.encoding})", flush=True)

    def on_client_unadvertise(self, client: Client, client_channel_id: int) -> None:
        self._topics.pop((client.id, client_channel_id), None)

    def on_message_data(self, client: Client, client_channel_id: int, data: bytes) -> None:
        topic = self._topics.get((client.id, client_channel_id))
        if topic != TOPIC_CMD_VEL:
            return
        self.drive(data)

    def drive(self, data: bytes) -> None:
        """Apply one ``/cmd_vel`` Twist, from either connection."""
        try:
            payload = json.loads(data)
        except ValueError:
            print("ignoring non-JSON /cmd_vel message", flush=True)
            return
        vx, vy, wz = twist_from_json(payload)
        if self._nav is not None and self._nav.active:
            self._nav.stop("teleop")
        self._loco.move(vx, vy, wz)
        if self._loco.paused or self._loco.mode == MODE_DAMP:
            return
        now = time.time_ns()
        with self._log_lock:
            foxglove.log(
                TOPIC_CMD_VEL, {"linear": {"x": vx, "y": vy}, "angular": {"z": wz}}, log_time=now
            )
            foxglove.log(TOPIC_LOCO_MODE, {"mode": MODE_WALK}, log_time=now)

    # The remaining ServerListener callbacks keep their no-op defaults.
    def on_subscribe(self, client: Client, channel: object) -> None:
        return None

    def on_unsubscribe(self, client: Client, channel: object) -> None:
        return None


class RemoteListener(RemoteAccessListener):
    """Remote Access callbacks. The gateway hands over the channel, topic included."""

    def __init__(self, app: AppListener) -> None:
        self._app = app

    def on_connection_status_changed(self, status: RemoteAccessConnectionStatus) -> None:
        print(f"remote access {status.name.lower()}", flush=True)

    def on_client_advertise(self, client: RemoteClient, channel: ChannelDescriptor) -> None:
        print(f"remote client {client.id} advertised {channel.topic}", flush=True)

    def on_message_data(
        self, client: RemoteClient, channel: ChannelDescriptor, data: bytes
    ) -> None:
        if channel.topic == TOPIC_CMD_VEL:
            self._app.drive(data)


def asset_handler(directory: Path):
    """Serve ``package://<name>/<path>`` from ``directory`` for the 3D panel.

    Only the path after the package name is used, so the URDF's relative
    ``meshes/*.STL`` resolve beside it. Nothing outside ``directory`` is served.
    """
    root = directory.resolve()

    def fetch(uri: str) -> bytes | None:
        if not uri.startswith(PACKAGE_SCHEME):
            return None
        _package, _, relative = uri[len(PACKAGE_SCHEME) :].partition("/")
        path = (root / relative).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            return None
        return path.read_bytes()

    return fetch


def build_services(
    loco: LocoBackend,
    sim: SimControl,
    log_lock: threading.Lock,
    nav: NavController | None = None,
) -> list[Service]:
    """Service Call panel entries. Each returns ``{"ok": ..., "detail": ...}``.

    ``ok`` is false only when a request was refused, such as ``nav/start``
    before the walking policy is running.
    """

    def service(name: str, action, detail, mode=None) -> Service:
        """``detail`` and ``mode`` may be callables, read after the action runs.

        ``action`` may return ``(ok, detail)`` to override both.
        """

        def handler(_request: ServiceRequest) -> bytes:
            result = action()
            print(f"service {name}", flush=True)
            ok = True
            text = detail() if callable(detail) else detail
            if isinstance(result, tuple):
                ok, text = result
            logged_mode = mode() if callable(mode) else mode
            if ok and logged_mode is not None:
                with log_lock:
                    foxglove.log(TOPIC_LOCO_MODE, {"mode": logged_mode}, log_time=time.time_ns())
            return json.dumps({"ok": ok, "detail": text}).encode()

        return Service(
            name=name,
            schema=ServiceSchema(name=name, request=_EMPTY_SCHEMA, response=_ACK_SCHEMA),
            handler=handler,
        )

    nav_services: list[Service] = []
    if nav is not None:
        nav_services = [
            service("nav/start", nav.start, None),
            service("nav/stop", nav.stop, None),
        ]

    def stop_nav_then(action):
        def run():
            if nav is not None and nav.active:
                nav.stop("mode change")
            action()

        return run

    return nav_services + [
        service("loco/stand", stop_nav_then(loco.stand), "ramping to the default pose", MODE_STAND),
        service("loco/start", stop_nav_then(loco.start), "walking policy running", MODE_WALK),
        service(
            "loco/pause",
            stop_nav_then(loco.pause),
            lambda: (
                "paused: balancing at zero velocity, Teleop ignored"
                if loco.paused
                else "not walking, nothing to pause"
            ),
            lambda: LOG_MODE_PAUSED if loco.paused else None,
        ),
        service("loco/resume", loco.resume, "Teleop drives again", MODE_WALK),
        service("loco/damp", stop_nav_then(loco.damp), "motors damped", MODE_DAMP),
        service("sim/band_release", sim.band_release, "elastic band off"),
        service("sim/band_attach", sim.band_attach, "elastic band on"),
        service("sim/band_lower", sim.band_lower, f"band {BAND_STEP_M:.1f} m longer"),
        service("sim/band_raise", sim.band_raise, f"band {BAND_STEP_M:.1f} m shorter"),
        service("sim/reset", stop_nav_then(sim.reset), "simulation reset to the initial pose"),
    ]


def main() -> None:
    """Serve and record until SIGTERM or the optional duration."""
    domain_id = int(os.environ.get("UNITREE_DOMAIN_ID", "1"))
    interface = os.environ.get("UNITREE_INTERFACE", "lo")
    duration_s = float(os.environ.get("UNITREE_RECORD_DURATION", "0"))
    host = os.environ.get("FOXGLOVE_HOST", "127.0.0.1")
    port = int(os.environ.get("FOXGLOVE_PORT", "8765"))
    try:
        profile = profile_for(os.environ.get("UNITREE_ROBOT", DEFAULT_ROBOT))
    except KeyError as exc:
        print(f"{exc} (SETUP-002)", file=sys.stderr)
        raise SystemExit(64) from exc
    output = _output_path(profile)
    try:
        world = world_for(os.environ.get("UNITREE_WORLD", DEFAULT_WORLD))
    except KeyError as exc:
        print(f"{exc} (SETUP-002)", file=sys.stderr)
        raise SystemExit(64) from exc

    stop = threading.Event()

    def handler(signum: int, _frame: object) -> None:
        print(f"stopping bridge on signal {signum}", flush=True)
        stop.set()

    signal.signal(signal.SIGTERM, handler)
    signal.signal(signal.SIGINT, handler)

    ChannelFactoryInitialize(domain_id, interface)
    low_sub = ChannelSubscriber("rt/lowstate", profile.low_state_type())
    sport_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    low_sub.Init()
    sport_sub.Init()
    loco = SimLocoBackend()
    sim = SimControl()
    log_lock = threading.Lock()
    health = HealthMonitor(THRESHOLDS[profile.name], time.time_ns())
    incidents = IncidentRecorder(output, time.time_ns())
    nav = NavController(loco, _nav_publisher(log_lock))
    nav.run_in_thread()
    scan_sub = ChannelSubscriber(DDS_TOPIC_SCAN, LaserScan)
    scan_sub.Init(_scan_handler(nav, log_lock), 2)

    app_listener = AppListener(loco, log_lock, nav)
    server = foxglove.start_server(
        name=f"unitree-{profile.name}",
        host=host,
        port=port,
        capabilities=[Capability.ClientPublish, Capability.Services],
        supported_encodings=["json"],
        services=build_services(loco, sim, log_lock, nav),
        server_listener=app_listener,
    )
    print(f"live Foxglove connection ws://{host}:{port}", flush=True)
    rl_gym_root = Path(os.environ.get("UNITREE_RL_GYM_ROOT", "third_party/unitree_rl_gym"))
    assets = start_asset_server(
        profile,
        host,
        int(os.environ.get("ASSETS_PORT", str(DEFAULT_ASSETS_PORT))),
        rl_gym_root,
    )
    gateway = start_remote_access(
        profile, rl_gym_root, build_services(loco, sim, log_lock, nav), app_listener
    )
    print(f"recording {output}", flush=True)
    if world is not None:
        print(
            f"world {world.name}: {world.length_m:.0f} m hallway drawn on {TOPIC_WORLD_SCENE}",
            flush=True,
        )

    sport_state = None
    saw_sample = False
    started = time.perf_counter()
    scene_due = started
    roll_s = 0.0 if os.environ.get("UNITREE_RECORD_PATH") else _record_roll_s()
    roll_at = started + roll_s if roll_s else None
    recording = foxglove.open_mcap(output)
    recording.__enter__()
    try:
        try:
            nav.announce()
            while not stop.is_set():
                if duration_s and time.perf_counter() - started >= duration_s:
                    break
                if roll_at is not None and time.perf_counter() >= roll_at:
                    recording.__exit__(None, None, None)
                    output = _output_path(profile)
                    incidents.recording = output
                    print(f"recording {output}", flush=True)
                    recording = foxglove.open_mcap(output)
                    recording.__enter__()
                    roll_at += roll_s
                if world is not None and time.perf_counter() >= scene_due:
                    _log_scene(world, log_lock)
                    scene_due += SCENE_REPUBLISH_S
                incoming_sport = sport_sub.Read(_READ_TIMEOUT_S)
                if _has(incoming_sport, "position"):
                    sport_state = incoming_sport
                low_state = low_sub.Read(_READ_TIMEOUT_S)
                incidents.update(health.current, time.time_ns())
                if not _has(low_state, "motor_state"):
                    _log_health(health.tick(time.time_ns()), log_lock, nav, loco)
                    continue
                if not saw_sample:
                    saw_sample = True
                    print("first sample received", flush=True)
                with log_lock:
                    nanoseconds = _log_sample(low_state, sport_state, profile)
                observation = observation_from_state(
                    low_state, sport_state, loco, sim.band_enabled, nanoseconds
                )
                _log_health(health.observe(observation), log_lock, nav, loco)
        finally:
            recording.__exit__(None, None, None)
    finally:
        nav.close()
        incidents.finish(time.time_ns())
        server.stop()
        if gateway is not None:
            gateway.stop()
        if assets is not None:
            assets.shutdown()
    print(f"recording closed {output}", flush=True)
    os._exit(0)


def mode_label(loco: LocoBackend) -> str:
    """The ``/loco/mode`` label: the DDS mode, or ``paused`` on top of ``walk``."""
    return LOG_MODE_PAUSED if loco.paused else loco.mode


def observation_from_state(
    low_state: object,
    sport_state: object | None,
    loco: LocoBackend,
    band_enabled: bool,
    nanoseconds: int,
) -> Observation:
    """What the health monitor judges: pose from DDS, intent from the bridge."""
    height = None if sport_state is None else float(sport_state.position[2])  # type: ignore[attr-defined]
    return Observation(
        time_ns=nanoseconds,
        height_m=height,
        tilt_deg=tilt_from_quaternion(low_state.imu_state.quaternion),  # type: ignore[attr-defined]
        mode=mode_label(loco),
        band_enabled=band_enabled,
    )


def _log_health(
    state: HealthState | None,
    log_lock: threading.Lock,
    nav: NavController | None = None,
    loco: LocoBackend | None = None,
) -> None:
    """Log a health change. Entering ``incident`` damps the motors.

    The navigator is stopped first, then damp. Damp sticks: a velocity command
    does not leave it, so a tick already in flight cannot restart the policy.
    Repeating the incident state does not send damp again.
    """
    if state is None:
        return
    if state.state == INCIDENT and loco is not None and loco.mode != MODE_DAMP:
        if nav is not None and nav.active:
            nav.stop("health incident")
        loco.damp()
        print("health incident: motors damped", flush=True)
        with log_lock:
            foxglove.log(TOPIC_LOCO_MODE, {"mode": MODE_DAMP}, log_time=time.time_ns())
    elif state.state == INCIDENT and nav is not None and nav.active:
        nav.stop("health incident")
    with log_lock:
        foxglove.log(TOPIC_HEALTH_STATE, state.to_json(), log_time=time.time_ns())


def _log_scene(world: World, log_lock: threading.Lock) -> None:
    now = time.time_ns()
    with log_lock:
        foxglove.log(
            TOPIC_WORLD_SCENE, scene_update_from_world(world, timestamp_from_ns(now)), log_time=now
        )


def _nav_publisher(log_lock: threading.Lock):
    def publish(decision: dict) -> None:
        with log_lock:
            foxglove.log(TOPIC_NAV_STATE, decision, log_time=time.time_ns())

    return publish


def _scan_handler(nav: NavController, log_lock: threading.Lock):
    """DDS callback: relay the scan to Foxglove and hand it to the navigator."""

    def on_scan(scan: object) -> None:
        if not _has(scan, "ranges"):
            return
        nav.on_scan(scan)
        now = time.time_ns()
        message = laser_scan_from_dds(scan, timestamp_from_ns(now))
        with log_lock:
            foxglove.log(TOPIC_SCAN, message, log_time=now)

    return on_scan


def _log_sample(low_state: object, sport_state: object | None, profile: RobotProfile) -> int:
    """Log one state sample; returns its log time in nanoseconds."""
    nanoseconds = time.time_ns()
    timestamp = timestamp_from_ns(nanoseconds)
    foxglove.log(
        TOPIC_JOINT_STATES,
        joint_states_from_lowstate(low_state, timestamp, profile),
        log_time=nanoseconds,
    )
    gyro, accel = imu_from_lowstate(low_state)
    foxglove.log(TOPIC_IMU_GYRO, gyro, log_time=nanoseconds)
    foxglove.log(TOPIC_IMU_ACCEL, accel, log_time=nanoseconds)
    if sport_state is None:
        return nanoseconds
    foxglove.log(
        TOPIC_BASE_POSE,
        base_pose_from_state(low_state, sport_state, timestamp),
        log_time=nanoseconds,
    )
    foxglove.log(
        TOPIC_BASE_VELOCITY,
        base_velocity_from_sport(sport_state),
        log_time=nanoseconds,
    )
    foxglove.log(
        TOPIC_TF,
        base_transform_from_state(low_state, sport_state, timestamp, profile),
        log_time=nanoseconds,
    )
    return nanoseconds


def start_asset_server(
    profile: RobotProfile, host: str, port: int, rl_gym_root: Path
) -> ThreadingHTTPServer | None:
    """Serve the URDF and meshes for the 3D panel; ``None`` when the robot has none."""
    directory = profile.description_path(rl_gym_root)
    if directory is None or profile.urdf_file is None:
        return None
    if not (directory / profile.urdf_file).is_file():
        print(f"no URDF at {directory / profile.urdf_file}; 3D model disabled", flush=True)
        return None
    server = serve_directory(directory, host, port)
    print(f"robot model http://{host}:{port}/{profile.urdf_file}", flush=True)
    return server


def start_remote_access(
    profile: RobotProfile,
    rl_gym_root: Path,
    services: list[Service],
    app_listener: AppListener,
) -> RemoteAccessGateway | None:
    """Start the Remote Access gateway when a device token is set; ``None`` otherwise.

    The gateway is a second sink on the same context as the WebSocket server, so
    it sees every channel. The token is read by the SDK from the environment.
    """
    if not os.environ.get("FOXGLOVE_DEVICE_TOKEN", "").strip():
        return None
    directory = profile.description_path(rl_gym_root)
    gateway = foxglove.start_gateway(
        capabilities=[RemoteCapability.ClientPublish, RemoteCapability.Services],
        listener=RemoteListener(app_listener),
        supported_encodings=["json"],
        services=services,
        asset_handler=None if directory is None else asset_handler(directory),
    )
    print("remote access gateway started", flush=True)
    return gateway


def _record_roll_s() -> float:
    """Seconds per telemetry MCAP. ``0`` keeps one file for the whole run."""
    return float(os.environ.get("UNITREE_RECORD_ROLL_S", "0"))


def _output_path(profile: RobotProfile) -> Path:
    override = os.environ.get("UNITREE_RECORD_PATH")
    if override:
        path = Path(override)
        path.parent.mkdir(parents=True, exist_ok=True)
        return path
    directory = Path(os.environ.get("RECORDINGS_DIR", "recordings"))
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return directory / f"{profile.name}-{stamp}.mcap"


def _has(sample: object, field: str) -> bool:
    if sample is None or not hasattr(sample, field):
        return False
    return type(sample).__name__ != "InvalidSample"


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
