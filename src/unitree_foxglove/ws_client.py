"""Minimal Foxglove WebSocket client: publish ``/cmd_vel`` and call services.

Speaks the Foxglove WebSocket protocol the same way the Foxglove app does, so
the headless end-to-end test exercises the exact server path the Teleop and
Service Call panels use. The SDK server (0.27) negotiates subprotocol
``foxglove.sdk.v1``; the JSON and binary frames match the published
``foxglove.websocket.v1`` layout.

``python -m unitree_foxglove.ws_client`` runs the stand-up walkthrough and a
short forward drive; the e2e script uses it and then checks the recording.
``--fall`` releases the rope without the policy; ``--nav`` hands the robot to
the hallway navigator and waits for it to arrive.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import struct
import sys

import websockets

SUBPROTOCOL = "foxglove.sdk.v1"
_OP_SERVER_MESSAGE_DATA = 0x01
_OP_CLIENT_MESSAGE = 0x01
_OP_SERVICE_CALL_REQUEST = 0x02
_OP_SERVICE_CALL_RESPONSE = 0x03
_CMD_VEL_CHANNEL_ID = 1
NAV_TERMINAL_STATES = ("arrived", "blind", "stopped")


class FoxgloveClient:
    """Async client for one server. Use as ``async with``."""

    def __init__(self, url: str) -> None:
        self.url = url
        self._socket = None
        self._services: dict[str, int] = {}
        self._channels: dict[str, int] = {}
        self._subscriptions: dict[int, str] = {}
        self._latest: dict[str, bytes] = {}
        self._next_call_id = 1
        self._next_subscription_id = 1
        self._responses: dict[int, asyncio.Future[bytes]] = {}
        self._reader: asyncio.Task[None] | None = None

    async def __aenter__(self) -> FoxgloveClient:
        self._socket = await websockets.connect(self.url, subprotocols=[SUBPROTOCOL])
        self._reader = asyncio.create_task(self._read_loop())
        await self._wait_for_services()
        await self._advertise_cmd_vel()
        return self

    async def __aexit__(self, *_exc: object) -> None:
        if self._reader is not None:
            self._reader.cancel()
        if self._socket is not None:
            await self._socket.close()

    async def _wait_for_services(self, timeout_s: float = 10.0) -> None:
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout_s
        while not self._services:
            if self._reader is not None and self._reader.done():
                self._reader.result()  # re-raise the reader's failure
                raise ConnectionError("server closed the connection")
            if loop.time() > deadline:
                raise TimeoutError("server advertised no services")
            await asyncio.sleep(0.05)

    async def _advertise_cmd_vel(self) -> None:
        assert self._socket is not None
        await self._socket.send(
            json.dumps(
                {
                    "op": "advertise",
                    "channels": [
                        {
                            "id": _CMD_VEL_CHANNEL_ID,
                            "topic": "/cmd_vel",
                            "encoding": "json",
                            "schemaName": "geometry_msgs/Twist",
                        }
                    ],
                }
            )
        )

    async def _read_loop(self) -> None:
        assert self._socket is not None
        async for frame in self._socket:
            if isinstance(frame, bytes):
                self._on_binary(frame)
                continue
            message = json.loads(frame)
            if message.get("op") == "advertiseServices":
                for service in message["services"]:
                    self._services[service["name"]] = int(service["id"])
            elif message.get("op") == "advertise":
                for channel in message["channels"]:
                    self._channels[channel["topic"]] = int(channel["id"])
            elif message.get("op") == "serviceCallFailure":
                future = self._responses.pop(int(message["callId"]), None)
                if future is not None and not future.done():
                    future.set_exception(RuntimeError(message.get("message", "service failed")))

    def _on_binary(self, frame: bytes) -> None:
        if frame[0] == _OP_SERVER_MESSAGE_DATA:
            (subscription_id,) = struct.unpack_from("<I", frame, 1)
            topic = self._subscriptions.get(subscription_id)
            if topic is not None:
                self._latest[topic] = frame[13:]  # after the 8-byte receive timestamp
            return
        if frame[0] != _OP_SERVICE_CALL_RESPONSE:
            return
        _service_id, call_id, encoding_length = struct.unpack_from("<III", frame, 1)
        payload = frame[13 + encoding_length :]
        future = self._responses.pop(call_id, None)
        if future is not None and not future.done():
            future.set_result(payload)

    @property
    def services(self) -> list[str]:
        return sorted(self._services)

    async def subscribe(self, topic: str, timeout_s: float = 10.0) -> None:
        """Subscribe to a server topic once the server has advertised it."""
        assert self._socket is not None
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout_s
        while topic not in self._channels:
            if loop.time() > deadline:
                raise TimeoutError(f"server never advertised {topic}")
            await asyncio.sleep(0.05)
        subscription_id = self._next_subscription_id
        self._next_subscription_id += 1
        self._subscriptions[subscription_id] = topic
        await self._socket.send(
            json.dumps(
                {
                    "op": "subscribe",
                    "subscriptions": [{"id": subscription_id, "channelId": self._channels[topic]}],
                }
            )
        )

    def latest_json(self, topic: str) -> dict | None:
        """The most recent JSON message received on a subscribed topic."""
        payload = self._latest.get(topic)
        return None if payload is None else json.loads(payload)

    async def call(self, name: str, request: dict | None = None, timeout_s: float = 5.0) -> dict:
        """Invoke a JSON service and return its decoded response."""
        assert self._socket is not None
        service_id = self._services[name]
        call_id = self._next_call_id
        self._next_call_id += 1
        future: asyncio.Future[bytes] = asyncio.get_running_loop().create_future()
        self._responses[call_id] = future
        encoding = b"json"
        payload = json.dumps(request or {}).encode()
        frame = (
            struct.pack("<BIII", _OP_SERVICE_CALL_REQUEST, service_id, call_id, len(encoding))
            + encoding
            + payload
        )
        await self._socket.send(frame)
        response = await asyncio.wait_for(future, timeout_s)
        return json.loads(response)

    async def publish_twist(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> None:
        """Send one ``geometry_msgs/Twist`` on ``/cmd_vel`` as the Teleop panel does."""
        assert self._socket is not None
        twist = {"linear": {"x": vx, "y": vy, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": wz}}
        frame = struct.pack("<BI", _OP_CLIENT_MESSAGE, _CMD_VEL_CHANNEL_ID)
        await self._socket.send(frame + json.dumps(twist).encode())

    async def drive(self, vx: float, wz: float, seconds: float, rate_hz: float = 10.0) -> None:
        """Repeat a Twist for ``seconds``; the controller zeroes velocity 0.5 s after the last."""
        period = 1.0 / rate_hz
        steps = max(1, int(seconds * rate_hz))
        for _ in range(steps):
            await self.publish_twist(vx=vx, wz=wz)
            await asyncio.sleep(period)
        await self.publish_twist()


async def stand_up_and_walk(
    url: str,
    *,
    band_lowers: int,
    forward_speed: float,
    forward_seconds: float,
    settle_s: float,
    pause_seconds: float = 0.0,
) -> None:
    """The README walkthrough as a script: stand, start, lower, release, drive.

    The policy must be balancing before the feet take load; a robot holding a
    fixed pose topples as the band goes slack.
    """
    async with FoxgloveClient(url) as client:
        print(f"services: {', '.join(client.services)}", flush=True)
        print(await client.call("loco/stand"), flush=True)
        await asyncio.sleep(settle_s)
        print(await client.call("loco/start"), flush=True)
        await asyncio.sleep(settle_s)
        for _ in range(band_lowers):
            await client.call("sim/band_lower")
            await asyncio.sleep(0.4)
        await asyncio.sleep(settle_s)
        print(await client.call("sim/band_release"), flush=True)
        await asyncio.sleep(settle_s)
        print(f"driving vx={forward_speed} for {forward_seconds}s", flush=True)
        await client.drive(forward_speed, 0.0, forward_seconds)
        await asyncio.sleep(1.0)
        print("drive done", flush=True)
        if pause_seconds > 0:
            # Teleop while paused must not move the robot, and it must stay upright.
            print(await client.call("loco/pause"), flush=True)
            await client.drive(forward_speed, 0.0, pause_seconds)
            print(await client.call("loco/resume"), flush=True)
            await asyncio.sleep(1.0)
            print("pause done", flush=True)


async def walk_the_hallway(url: str, *, band_lowers: int, settle_s: float, timeout_s: float) -> str:
    """Stand, release, then let the navigator walk to the end of the hallway.

    Returns the navigator's final state. ``arrived`` is success; anything else
    means the robot stopped short, so the e2e fails before checking the MCAP.
    """
    async with FoxgloveClient(url) as client:
        print(f"services: {', '.join(client.services)}", flush=True)
        await client.subscribe("/nav/state")
        print(await client.call("loco/stand"), flush=True)
        await asyncio.sleep(settle_s)
        print(await client.call("loco/start"), flush=True)
        await asyncio.sleep(settle_s)
        for _ in range(band_lowers):
            await client.call("sim/band_lower")
            await asyncio.sleep(0.4)
        await asyncio.sleep(settle_s)
        print(await client.call("sim/band_release"), flush=True)
        await asyncio.sleep(settle_s)
        started = await client.call("nav/start")
        print(started, flush=True)
        if not started.get("ok"):
            return "refused"
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout_s
        report_at = loop.time()
        state = "walking"
        while loop.time() < deadline:
            latest = client.latest_json("/nav/state")
            if latest is not None:
                state = latest["state"]
                if loop.time() >= report_at:
                    print(
                        f"nav {state}: front={latest['front_m']} left={latest['left_m']} "
                        f"right={latest['right_m']} heading={latest['heading_deg']:.1f} deg",
                        flush=True,
                    )
                    report_at += 2.0
                if state in NAV_TERMINAL_STATES:
                    break
            await asyncio.sleep(0.2)
        else:
            print(f"nav still {state} after {timeout_s:.0f}s", flush=True)
            await client.call("nav/stop")
            return "timeout"
        await asyncio.sleep(1.0)
        print(f"nav finished: {state}", flush=True)
        return state


async def fall_without_the_policy(
    url: str, *, band_lowers: int, settle_s: float, after_release_s: float
) -> None:
    """Stand on the rope and release without the walking policy, so the robot falls.

    Used after a walk in the same simulator: damp and reset first, because the
    policy may still be running and the rope may already be off.
    """
    async with FoxgloveClient(url) as client:
        print(f"services: {', '.join(client.services)}", flush=True)
        print(await client.call("loco/damp"), flush=True)
        print(await client.call("sim/reset"), flush=True)
        await asyncio.sleep(settle_s)
        print(await client.call("loco/stand"), flush=True)
        await asyncio.sleep(settle_s)
        for _ in range(band_lowers):
            await client.call("sim/band_lower")
            await asyncio.sleep(0.4)
        await asyncio.sleep(settle_s)
        print(await client.call("sim/band_release"), flush=True)
        print(f"released; waiting {after_release_s:.0f}s for the incident window", flush=True)
        await asyncio.sleep(after_release_s)
        print("fall done", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--url",
        default=f"ws://{os.environ.get('FOXGLOVE_HOST', '127.0.0.1')}:"
        f"{os.environ.get('FOXGLOVE_PORT', '8765')}",
    )
    parser.add_argument("--band-lowers", type=int, default=7)
    parser.add_argument("--speed", type=float, default=0.4)
    parser.add_argument("--seconds", type=float, default=4.0)
    parser.add_argument("--settle", type=float, default=3.0)
    parser.add_argument("--pause-seconds", type=float, default=3.0)
    parser.add_argument(
        "--fall",
        action="store_true",
        help="release without the walking policy and wait for the incident window",
    )
    parser.add_argument("--after-release", type=float, default=20.0)
    parser.add_argument(
        "--nav",
        action="store_true",
        help="stand up, then nav/start and wait until the navigator reaches the end wall",
    )
    parser.add_argument("--nav-timeout", type=float, default=120.0)
    args = parser.parse_args()
    try:
        if args.nav:
            state = asyncio.run(
                walk_the_hallway(
                    args.url,
                    band_lowers=args.band_lowers,
                    settle_s=args.settle,
                    timeout_s=args.nav_timeout,
                )
            )
            if state != "arrived":
                print(f"navigator ended in {state!r}, not 'arrived' (E2E-002)", file=sys.stderr)
                raise SystemExit(2)
        elif args.fall:
            asyncio.run(
                fall_without_the_policy(
                    args.url,
                    band_lowers=args.band_lowers,
                    settle_s=args.settle,
                    after_release_s=args.after_release,
                )
            )
        else:
            asyncio.run(
                stand_up_and_walk(
                    args.url,
                    band_lowers=args.band_lowers,
                    forward_speed=args.speed,
                    forward_seconds=args.seconds,
                    settle_s=args.settle,
                    pause_seconds=args.pause_seconds,
                )
            )
    except (OSError, TimeoutError) as exc:
        print(f"could not drive via {args.url}: {exc} (E2E-001)", file=sys.stderr)
        raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
