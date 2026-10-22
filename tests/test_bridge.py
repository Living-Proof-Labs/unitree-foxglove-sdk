"""Teleop messages and service calls reach the locomotion backend.

DDS publishers are replaced by fakes; no CycloneDDS participant is created.
"""

import json
import threading
from types import SimpleNamespace

import pytest

from unitree_foxglove import bridge
from unitree_foxglove.bridge import LOG_MODE_PAUSED
from unitree_foxglove.idl import MODE_DAMP, MODE_STAND, MODE_WALK
from unitree_foxglove.sim import BAND_STEP_M


class FakeLoco:
    def __init__(self) -> None:
        self.calls: list[tuple] = []
        self.paused = False
        self.mode = MODE_WALK

    def damp(self) -> None:
        self.calls.append(("damp",))
        self.mode = MODE_DAMP

    def stand(self) -> None:
        self.calls.append(("stand",))

    def start(self) -> None:
        self.calls.append(("start",))

    def pause(self) -> None:
        self.calls.append(("pause",))
        self.paused = True

    def resume(self) -> None:
        self.calls.append(("resume",))
        self.paused = False

    def move(self, vx: float, vy: float, wz: float) -> None:
        self.calls.append(("move", vx, vy, wz))


class FakePublisher:
    def __init__(self, topic: str, _type: type) -> None:
        self.topic = topic
        self.written: list[object] = []

    def Init(self) -> None:  # noqa: N802 - unitree_sdk2py naming
        pass

    def Write(self, message: object) -> None:  # noqa: N802
        self.written.append(message)


@pytest.fixture
def logged(monkeypatch: pytest.MonkeyPatch) -> list[tuple[str, dict]]:
    records: list[tuple[str, dict]] = []
    monkeypatch.setattr(
        bridge.foxglove, "log", lambda topic, msg, log_time=None: records.append((topic, msg))
    )
    return records


def test_cmd_vel_from_an_advertised_client_channel_moves_the_robot(logged: list) -> None:
    loco = FakeLoco()
    listener = bridge.AppListener(loco, threading.Lock())
    client = SimpleNamespace(id=7)
    listener.on_client_advertise(client, SimpleNamespace(id=3, topic="/cmd_vel", encoding="json"))
    listener.on_client_advertise(client, SimpleNamespace(id=4, topic="/other", encoding="json"))

    twist = {"linear": {"x": 0.4, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": -0.6}}
    listener.on_message_data(client, 3, json.dumps(twist).encode())
    listener.on_message_data(client, 4, json.dumps(twist).encode())
    listener.on_message_data(client, 3, b"not json")

    assert loco.calls == [("move", 0.4, 0.0, -0.6)]
    assert logged == [
        ("/cmd_vel", {"linear": {"x": 0.4, "y": 0.0}, "angular": {"z": -0.6}}),
        ("/loco/mode", {"mode": MODE_WALK}),
    ]

    # While paused the Twist reaches the backend, which drops it, and is not
    # logged as a walk command.
    loco.paused = True
    listener.on_message_data(client, 3, json.dumps(twist).encode())
    assert len(logged) == 2
    loco.paused = False

    listener.on_client_unadvertise(client, 3)
    listener.on_message_data(client, 3, json.dumps(twist).encode())
    assert len(loco.calls) == 2


def test_remote_cmd_vel_moves_the_robot_by_topic(logged: list) -> None:
    loco = FakeLoco()
    remote = bridge.RemoteListener(bridge.AppListener(loco, threading.Lock()))
    client = SimpleNamespace(id=1)
    twist = {"linear": {"x": 0.2, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.3}}

    remote.on_message_data(client, SimpleNamespace(topic="/cmd_vel"), json.dumps(twist).encode())
    remote.on_message_data(client, SimpleNamespace(topic="/other"), json.dumps(twist).encode())

    assert loco.calls == [("move", 0.2, 0.0, 0.3)]
    assert logged[0] == ("/cmd_vel", {"linear": {"x": 0.2, "y": 0.0}, "angular": {"z": 0.3}})


def test_asset_handler_serves_the_description_directory_only(tmp_path) -> None:
    (tmp_path / "meshes").mkdir()
    (tmp_path / "g1.urdf").write_bytes(b"<robot/>")
    (tmp_path / "meshes" / "pelvis.STL").write_bytes(b"solid")
    (tmp_path.parent / "secret.txt").write_bytes(b"no")
    fetch = bridge.asset_handler(tmp_path)

    assert fetch("package://g1_description/g1.urdf") == b"<robot/>"
    assert fetch("package://g1_description/meshes/pelvis.STL") == b"solid"
    assert fetch("package://g1_description/../secret.txt") is None
    assert fetch("package://g1_description/missing.STL") is None
    assert fetch("http://localhost:8770/g1.urdf") is None


def test_no_gateway_without_a_device_token(monkeypatch: pytest.MonkeyPatch, tmp_path) -> None:
    monkeypatch.setenv("FOXGLOVE_DEVICE_TOKEN", " ")
    monkeypatch.setattr(bridge.foxglove, "start_gateway", pytest.fail)
    profile = SimpleNamespace(description_path=lambda _root: tmp_path)
    listener = bridge.AppListener(FakeLoco(), threading.Lock())

    assert bridge.start_remote_access(profile, tmp_path, [], listener) is None


def test_services_call_the_backend_and_answer_ok(
    monkeypatch: pytest.MonkeyPatch, logged: list
) -> None:
    monkeypatch.setattr(bridge, "ChannelPublisher", FakePublisher)
    loco = FakeLoco()
    sim = bridge.SimControl()
    services = {
        service.name: service for service in bridge.build_services(loco, sim, threading.Lock())
    }

    assert set(services) == {
        "loco/stand",
        "loco/start",
        "loco/pause",
        "loco/resume",
        "loco/damp",
        "sim/band_release",
        "sim/band_attach",
        "sim/band_lower",
        "sim/band_raise",
        "sim/reset",
    }
    for name in ("loco/stand", "loco/start", "loco/pause", "loco/resume", "loco/damp"):
        response = json.loads(services[name].handler(SimpleNamespace(payload=b"{}")))
        assert response["ok"] is True
    assert loco.calls == [("stand",), ("start",), ("pause",), ("resume",), ("damp",)]
    assert [msg for topic, msg in logged if topic == "/loco/mode"] == [
        {"mode": MODE_STAND},
        {"mode": MODE_WALK},
        {"mode": LOG_MODE_PAUSED},
        {"mode": MODE_WALK},
        {"mode": MODE_DAMP},
    ]

    for name in ("sim/band_lower", "sim/band_raise", "sim/band_release", "sim/reset"):
        services[name].handler(SimpleNamespace(payload=b"{}"))
    published = sim._publisher.written  # type: ignore[attr-defined]
    assert [msg.band_length_delta for msg in published] == pytest.approx(
        [BAND_STEP_M, -BAND_STEP_M, 0.0, 0.0]
    )
    assert [msg.band_enable for msg in published] == [True, True, False, True]
    assert [msg.reset for msg in published] == [False, False, False, True]


def test_sim_loco_backend_publishes_modes(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(bridge, "ChannelPublisher", FakePublisher)
    backend = bridge.SimLocoBackend()
    backend.stand()
    backend.start()
    backend.move(0.3, 0.0, 0.2)
    backend.pause()
    backend.move(0.5, 0.0, 0.0)  # dropped while paused
    backend.resume()
    backend.move(0.1, 0.0, 0.0)
    backend.damp()
    written = backend._publisher.written  # type: ignore[attr-defined]
    assert [msg.mode for msg in written] == [
        MODE_STAND,
        MODE_WALK,
        MODE_WALK,
        MODE_WALK,
        MODE_WALK,
        MODE_DAMP,
    ]
    # Pause keeps the policy balancing: walk at zero velocity, not a new mode.
    assert (written[3].vx, written[3].vy, written[3].wz) == (0.0, 0.0, 0.0)
    assert written[4].vx == 0.1
    assert (written[2].vx, written[2].vy, written[2].wz) == (0.3, 0.0, 0.2)
    assert written[2].stamp_ns > 0
    assert backend.mode == MODE_DAMP
    backend.move(0.4, 0.0, 0.1)  # a Twist does not leave damp
    assert backend.mode == MODE_DAMP
    assert len(written) == 6


class FakeNav:
    def __init__(self, active: bool = False) -> None:
        self.active = active
        self.stops: list[str] = []
        self.starts = 0

    def start(self) -> tuple[bool, str]:
        self.starts += 1
        self.active = True
        return True, "walking"

    def stop(self, reason: str = "operator") -> tuple[bool, str]:
        self.stops.append(reason)
        self.active = False
        return True, "stopped"


def test_teleop_cancels_navigation(logged: list) -> None:
    loco = FakeLoco()
    nav = FakeNav(active=True)
    listener = bridge.AppListener(loco, threading.Lock(), nav)
    client = SimpleNamespace(id=1)
    listener.on_client_advertise(client, SimpleNamespace(id=3, topic="/cmd_vel", encoding="json"))
    twist = {"linear": {"x": 0.2}, "angular": {"z": 0.0}}
    listener.on_message_data(client, 3, json.dumps(twist).encode())
    assert nav.stops == ["teleop"]
    assert loco.calls == [("move", 0.2, 0.0, 0.0)]
    listener.on_message_data(client, 3, json.dumps(twist).encode())
    assert nav.stops == ["teleop"]  # already inactive: nothing more to cancel


def test_nav_services_and_mode_changes(monkeypatch: pytest.MonkeyPatch, logged: list) -> None:
    monkeypatch.setattr(bridge, "ChannelPublisher", FakePublisher)
    loco = FakeLoco()
    nav = FakeNav()
    services = {
        s.name: s for s in bridge.build_services(loco, bridge.SimControl(), threading.Lock(), nav)
    }
    assert {"nav/start", "nav/stop"} <= set(services)
    response = json.loads(services["nav/start"].handler(SimpleNamespace(payload=b"{}")))
    assert response == {"ok": True, "detail": "walking"}
    assert nav.active
    # Changing the locomotion mode while navigating hands control back first.
    services["loco/pause"].handler(SimpleNamespace(payload=b"{}"))
    assert nav.stops == ["mode change"]
    assert loco.calls[-1] == ("pause",)
    services["loco/stand"].handler(SimpleNamespace(payload=b"{}"))
    assert nav.stops == ["mode change"]  # inactive now, no second stop
    response = json.loads(services["nav/stop"].handler(SimpleNamespace(payload=b"{}")))
    assert response == {"ok": True, "detail": "stopped"}
    assert nav.stops[-1] == "operator"


def test_refused_service_answers_ok_false(monkeypatch: pytest.MonkeyPatch, logged: list) -> None:
    monkeypatch.setattr(bridge, "ChannelPublisher", FakePublisher)
    nav = FakeNav()
    nav.start = lambda: (False, "start the walking policy first")  # type: ignore[method-assign]
    services = {
        s.name: s
        for s in bridge.build_services(FakeLoco(), bridge.SimControl(), threading.Lock(), nav)
    }
    response = json.loads(services["nav/start"].handler(SimpleNamespace(payload=b"{}")))
    assert response["ok"] is False
    assert "walking policy" in response["detail"]


def test_health_incident_damps_the_motors_and_stops_navigation(logged: list) -> None:
    from unitree_foxglove.health import INCIDENT, NOMINAL, HealthState

    nav = FakeNav(active=True)
    loco = FakeLoco()
    bridge._log_health(HealthState(NOMINAL, "upright", 0), threading.Lock(), nav, loco)
    assert nav.stops == []
    assert loco.calls == []
    bridge._log_health(HealthState(INCIDENT, "fell", 1), threading.Lock(), nav, loco)
    assert nav.stops == ["health incident"]
    assert loco.calls == [("damp",)]
    assert loco.mode == MODE_DAMP
    bridge._log_health(HealthState(INCIDENT, "fell", 1), threading.Lock(), nav, loco)
    assert loco.calls == [("damp",)]  # already damped: no second command
    assert [topic for topic, _ in logged] == [
        "/health/state",
        "/loco/mode",
        "/health/state",
        "/health/state",
    ]
    assert logged[1] == ("/loco/mode", {"mode": MODE_DAMP})


def test_observation_reads_pose_and_the_requested_mode() -> None:
    low_state = SimpleNamespace(imu_state=SimpleNamespace(quaternion=[1.0, 0.0, 0.0, 0.0]))
    sport_state = SimpleNamespace(position=[0.2, 0.0, 0.74])
    loco = FakeLoco()
    loco.mode = MODE_WALK
    loco.paused = True

    observation = bridge.observation_from_state(low_state, sport_state, loco, False, 42)
    assert observation.height_m == pytest.approx(0.74)
    assert observation.tilt_deg == pytest.approx(0.0)
    assert observation.mode == LOG_MODE_PAUSED
    assert observation.time_ns == 42

    loco.paused = False
    assert bridge.observation_from_state(low_state, None, loco, True, 0).height_m is None
    assert bridge.mode_label(loco) == MODE_WALK
