"""DDS transport. Monitoring never creates a low-command publisher."""
from __future__ import annotations

import copy
import os
from pathlib import Path
import struct
import threading
import time
import numpy as np
from .robot_state import RobotState


def fill_motor_command(message, motor_indices, target_q, kp, kd, mode_machine):
    """Compact 23DoF vectors -> sparse HG motor slots; unused slots disabled."""
    arrays = [np.asarray(a, dtype=np.float64) for a in (target_q, kp, kd)]
    if len(set(motor_indices)) != 23 or any(not 0 <= i < 35 for i in motor_indices):
        raise ValueError('Expected 23 unique HG motor slots')
    if any(a.shape != (23,) or not np.isfinite(a).all() for a in arrays):
        raise ValueError('Expected finite compact 23DoF target and gains')
    if np.any(arrays[1] < 0) or np.any(arrays[2] < 0):
        raise ValueError('Motor gains must be nonnegative')
    message.mode_pr = 0
    message.mode_machine = int(mode_machine)
    for motor in message.motor_cmd:
        motor.mode = 0
        motor.q = motor.dq = motor.tau = motor.kp = motor.kd = 0.0
    for compact, slot in enumerate(motor_indices):
        motor = message.motor_cmd[slot]
        motor.mode = 1
        motor.q, motor.kp, motor.kd = (float(a[compact]) for a in arrays)
    return message


class UnitreeTransport:
    def __init__(self, network, domain_id, motor_indices, lowcmd_topic='rt/lowcmd',
                 lowstate_topic='rt/lowstate', enable_output=False):
        local_dds = Path(__file__).resolve().parents[1] / '.deps/cyclonedds'
        if local_dds.is_dir():
            os.environ.setdefault('CYCLONEDDS_HOME', str(local_dds))
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.utils.crc import CRC
        self.motor_indices = list(motor_indices)
        self.enable_output = bool(enable_output)
        self.lock = threading.Lock()
        self.latest = None
        self.last_error = None
        self.last_tick = None
        self.crc = CRC()
        self.command_factory = unitree_hg_msg_dds__LowCmd_
        self.publisher = None
        self.subscriber = None
        self._closed = False
        ChannelFactoryInitialize(domain_id, network)
        try:
            if self.enable_output:
                self.publisher = ChannelPublisher(lowcmd_topic, LowCmd_)
                self.publisher.Init()
            self.subscriber = ChannelSubscriber(lowstate_topic, LowState_)
            self.subscriber.Init(self._callback, 10)
        except BaseException:
            self.close()
            raise

    def _callback(self, message):
        try:
            if self.crc.Crc(message) != message.crc:
                raise ValueError('LowState CRC mismatch')
            if message.mode_pr != 0:
                raise ValueError('LowState is not in PR joint coordinates')
            tick = int(message.tick)
            with self.lock:
                if self.last_tick is not None:
                    advance = (tick - self.last_tick) & 0xffffffff
                    if advance == 0:
                        return  # Repeated packets must not refresh the watchdog.
                    if advance > 0x7fffffff:
                        raise ValueError('LowState tick moved backwards')
            motors = [message.motor_state[i] for i in self.motor_indices]
            remote = bytes(message.wireless_remote)
            buttons = struct.unpack_from('<H', remote, 2)[0]
            sticks = np.array([struct.unpack_from('<f', remote, offset)[0]
                               for offset in (4, 20, 8, 12)], np.float32)
            state = RobotState(
                q=np.array([m.q for m in motors], np.float32),
                dq=np.array([m.dq for m in motors], np.float32),
                quaternion_wxyz=np.array(message.imu_state.quaternion, np.float32),
                gyro=np.array(message.imu_state.gyroscope, np.float32),
                temperature_c=np.array([max(m.temperature) for m in motors], np.float32),
                motor_errors=np.array([m.motorstate for m in motors], np.uint32),
                buttons=buttons, sticks=sticks, mode_machine=int(message.mode_machine),
                tick=tick, received_at_ns=time.monotonic_ns(),
                motor_modes=np.array([m.mode for m in motors], np.uint8),
                tau_est=np.array([m.tau_est for m in motors], np.float32),
            )
            with self.lock:
                self.latest, self.last_tick = state, tick
        except Exception as exc:
            with self.lock:
                self.last_error = str(exc)  # Latched: callers must restart after a transport fault.

    def snapshot(self):
        with self.lock:
            if self.last_error:
                raise RuntimeError(self.last_error)
            return copy.deepcopy(self.latest)

    def release_control(self, timeout_s=5.0):
        if not self.enable_output:
            raise RuntimeError('Monitoring cannot release robot control')
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
        client = MotionSwitcherClient()
        client.SetTimeout(min(timeout_s, 2.0))
        client.Init()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            code, result = client.CheckMode()
            if code != 0 or not isinstance(result, dict) or 'name' not in result:
                raise RuntimeError(f'MotionSwitcher CheckMode failed: {code}, {result}')
            if not result['name']:
                return
            result = client.ReleaseMode()
            code = result[0] if isinstance(result, tuple) else result
            if code != 0:
                raise RuntimeError(f'MotionSwitcher ReleaseMode failed: {code}')
            time.sleep(0.1)
        raise TimeoutError('Robot control service did not release in time')

    def send(self, target_q, kp, kd, mode_machine):
        if self.publisher is None or self._closed:
            raise RuntimeError('No enabled LowCmd publisher')
        message = fill_motor_command(self.command_factory(), self.motor_indices,
                                     target_q, kp, kd, mode_machine)
        message.crc = self.crc.Crc(message)
        if not self.publisher.Write(message):
            raise RuntimeError('LowCmd publish failed')

    def send_damping(self, kd, mode_machine):
        self.send(np.zeros(23), np.zeros(23), np.full(23, kd), mode_machine)

    def close(self):
        self._closed = True
        if self.subscriber is not None:
            self.subscriber.Close()
        if self.publisher is not None:
            self.publisher.Close()
