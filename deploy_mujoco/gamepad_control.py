"""Nonblocking Linux joystick input for planar velocity commands."""
from __future__ import annotations

import os
import struct
import time
from collections.abc import Mapping, Sequence

import numpy as np

# linux/joystick.h ioctl requests and linux/input-event-codes.h axis codes.
_JSIOCGAXES = 0x80016A11
_JSIOCGAXMAP = 0x80406A32
_JSIOCGNAME = 0x80806A13
_EVENT = struct.Struct('=IhBB')
_AXIS = 0x02
_INIT = 0x80
_ABS_X, _ABS_Y, _ABS_RX = 0x00, 0x01, 0x03


def axis_velocity(value: float, limits: Sequence[float], deadzone: float) -> float:
    """Apply a continuous deadzone, then scale each direction independently."""
    value = float(np.clip(value, -1.0, 1.0))
    if abs(value) <= deadzone:
        return 0.0
    magnitude = (abs(value) - deadzone) / (1.0 - deadzone)
    return magnitude * float(limits[1] if value > 0 else limits[0])


class GamepadControl:
    """Left Y -> vx, left X -> vy, right X -> yaw rate; no vertical command.

    A newly connected controller must first return all three axes to neutral.
    Missing/disconnected devices hold zero commands and retry once per second.
    """

    def __init__(self, ranges: Mapping[str, Sequence[float]], *,
                 device: str = '/dev/input/js0', deadzone: float = 0.12):
        if not 0.0 <= deadzone < 1.0:
            raise ValueError('gamepad deadzone must be in [0, 1)')
        self.ranges = [tuple(ranges[key]) for key in ('lin_vel_x', 'lin_vel_y', 'ang_vel_z')]
        for limits in self.ranges:
            if len(limits) != 2 or not np.isfinite(limits).all() or not limits[0] <= 0 <= limits[1]:
                raise ValueError('gamepad velocity ranges must be finite and include zero')
        self.device = device
        self.deadzone = deadzone
        self._fd: int | None = None
        self._axis_indices: list[int] = []
        self._axes: dict[int, float] = {}
        self._armed = False
        self._retry_at = 0.0
        self._last_error: str | None = None

    def _connect(self) -> None:
        # Import lazily so keyboard control remains usable on other platforms.
        import fcntl

        fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
        try:
            count = bytearray(1)
            axis_map = bytearray(64)
            name = bytearray(128)
            fcntl.ioctl(fd, _JSIOCGAXES, count, True)
            fcntl.ioctl(fd, _JSIOCGAXMAP, axis_map, True)
            fcntl.ioctl(fd, _JSIOCGNAME, name, True)
            codes = list(axis_map[:count[0]])
            indices = [codes.index(code) for code in (_ABS_Y, _ABS_X, _ABS_RX)]
        except BaseException:
            os.close(fd)
            raise
        self._fd = fd
        self._axis_indices = indices
        self._axes.clear()
        self._armed = False
        self._last_error = None
        label = bytes(name).split(b'\0', 1)[0].decode(errors='replace')
        print(f'[INFO] Gamepad: {label} ({self.device}); axes vx/vy/yaw={indices}. '
              'Center both sticks to enable control.')

    def close(self) -> None:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
        self._axes.clear()
        self._armed = False

    def update(self, command: np.ndarray) -> None:
        command[:] = 0.0
        if self._fd is None:
            if time.monotonic() < self._retry_at:
                return
            try:
                self._connect()
            except (OSError, ValueError, ImportError) as exc:
                self._retry_at = time.monotonic() + 1.0
                message = str(exc)
                if message != self._last_error:
                    print(f'[WARN] Gamepad unavailable: {message}. Commands stay zero; retrying. '
                          'Use --control keyboard for keyboard input.')
                    self._last_error = message
                return
        try:
            for _ in range(256):
                try:
                    packet = os.read(self._fd, _EVENT.size)
                except BlockingIOError:
                    break
                if len(packet) != _EVENT.size:
                    raise OSError('joystick disconnected or returned an incomplete event')
                _, value, kind, number = _EVENT.unpack(packet)
                if (kind & ~_INIT) == _AXIS:
                    self._axes[number] = float(np.clip(value / 32767.0, -1.0, 1.0))
        except OSError as exc:
            print(f'[WARN] Gamepad disconnected: {exc}. Commands reset to zero.')
            self.close()
            self._retry_at = time.monotonic() + 1.0
            return
        if not all(index in self._axes for index in self._axis_indices):
            return
        # Linux sticks: up/left negative; robot: forward/left/CCW positive.
        values = [-self._axes[index] for index in self._axis_indices]
        if not self._armed:
            if any(abs(value) > self.deadzone for value in values):
                return
            self._armed = True
            print('[INFO] Gamepad ready: left stick moves x/y; right stick turns; release to stop.')
        command[:] = [axis_velocity(value, limits, self.deadzone)
                      for value, limits in zip(values, self.ranges)]
