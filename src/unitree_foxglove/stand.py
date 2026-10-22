"""Stand the simulated Go2 up, then lower it, without waiting for a keypress.

This is the upstream ``example/python/stand_go2.py`` trajectory. The original
blocks on ``input()``, which does not work as a one-shot container command.
"""

from __future__ import annotations

import os
import signal
import threading
import time

import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

_DT = 0.002
_STAND_UP_S = 3.0

_STAND_UP = np.array(
    [
        0.00571868,
        0.608813,
        -1.21763,
        -0.00571868,
        0.608813,
        -1.21763,
        0.00571868,
        0.608813,
        -1.21763,
        -0.00571868,
        0.608813,
        -1.21763,
    ],
    dtype=float,
)
_STAND_DOWN = np.array(
    [
        0.0473455,
        1.22187,
        -2.44375,
        -0.0473455,
        1.22187,
        -2.44375,
        0.0473455,
        1.22187,
        -2.44375,
        -0.0473455,
        1.22187,
        -2.44375,
    ],
    dtype=float,
)


def main() -> None:
    """Publish ``rt/lowcmd`` until UNITREE_STAND_DURATION elapses."""
    domain_id = int(os.environ.get("UNITREE_DOMAIN_ID", "1"))
    interface = os.environ.get("UNITREE_INTERFACE", "lo")
    duration_s = float(os.environ.get("UNITREE_STAND_DURATION", "8"))
    stop = threading.Event()

    def handler(signum: int, _frame: object) -> None:
        print(f"stopping stand command on signal {signum}", flush=True)
        stop.set()

    signal.signal(signal.SIGTERM, handler)
    signal.signal(signal.SIGINT, handler)

    ChannelFactoryInitialize(domain_id, interface)
    publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
    publisher.Init()
    command = _initial_command()
    crc = CRC()
    elapsed = 0.0
    print(
        f"publishing stand trajectory for {duration_s:.1f}s "
        f"on domain {domain_id} interface {interface}",
        flush=True,
    )
    while not stop.is_set() and elapsed < duration_s:
        step_start = time.perf_counter()
        _fill_targets(command, elapsed)
        command.crc = crc.Crc(command)
        publisher.Write(command)
        elapsed += _DT
        remaining = _DT - (time.perf_counter() - step_start)
        if remaining > 0:
            time.sleep(remaining)
    print("stand trajectory finished", flush=True)
    os._exit(0)


def _initial_command() -> LowCmd_:
    command = unitree_go_msg_dds__LowCmd_()
    command.head[0] = 0xFE
    command.head[1] = 0xEF
    command.level_flag = 0xFF
    command.gpio = 0
    for index in range(20):
        command.motor_cmd[index].mode = 0x01
        command.motor_cmd[index].q = 0.0
        command.motor_cmd[index].kp = 0.0
        command.motor_cmd[index].dq = 0.0
        command.motor_cmd[index].kd = 0.0
        command.motor_cmd[index].tau = 0.0
    return command


def _fill_targets(command: LowCmd_, elapsed: float) -> None:
    if elapsed < _STAND_UP_S:
        phase = float(np.tanh(elapsed / 1.2))
        start = _STAND_DOWN
        goal = _STAND_UP
        for index in range(12):
            command.motor_cmd[index].kp = phase * 50.0 + (1 - phase) * 20.0
    else:
        phase = float(np.tanh((elapsed - _STAND_UP_S) / 1.2))
        start = _STAND_UP
        goal = _STAND_DOWN
        for index in range(12):
            command.motor_cmd[index].kp = 50.0
    for index in range(12):
        command.motor_cmd[index].q = phase * goal[index] + (1 - phase) * start[index]
        command.motor_cmd[index].dq = 0.0
        command.motor_cmd[index].kd = 3.5
        command.motor_cmd[index].tau = 0.0
