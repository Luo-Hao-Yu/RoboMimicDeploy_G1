"""Hardware-independent low-state snapshot used by the 23DoF deployment."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class RobotState:
    # q/dq/temperature/errors are in the compact sdk_joint_names order (23).
    q: np.ndarray
    dq: np.ndarray
    quaternion_wxyz: np.ndarray
    gyro: np.ndarray
    temperature_c: np.ndarray
    motor_errors: np.ndarray
    # Unitree wireless_remote bit mask; sticks are lx, ly, rx, ry.
    buttons: int
    sticks: np.ndarray
    mode_machine: int
    tick: int
    received_at_ns: int
    # Optional diagnostic fields for old snapshots/tests; supplied by live DDS.
    motor_modes: np.ndarray | None = None
    tau_est: np.ndarray | None = None
