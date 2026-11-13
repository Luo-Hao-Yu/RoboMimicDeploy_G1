from __future__ import annotations

from pathlib import Path
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / 'deploy_real/config/g1_23dof.yaml'


def load_config(path=DEFAULT_CONFIG):
    path = Path(path).resolve()
    cfg = yaml.safe_load(path.read_text())
    if not isinstance(cfg, dict):
        raise ValueError('Expected real deployment configuration')
    control = cfg['control']
    defaults = dict(ready_joint_tolerance_rad=.25, ready_max_joint_velocity_rad_s=.2,
                    ready_max_gyro_rad_s=.2, ready_hold_s=.5, ready_timeout_s=3.)
    for key,value in defaults.items(): control.setdefault(key,value)
    positive = ('motor_dt', 'state_timeout_s', 'depth_timeout_s', 'action_timeout_s',
                'startup_timeout_s', 'ramp_s', *defaults, 'policy_blend_s', 'damping_kd', 'damping_duration_s',
                'max_tilt_rad', 'max_temperature_c', 'max_abs_action',
                'max_target_step_rad_s', 'max_position_error_rad', 'max_forward_m_s', 'max_yaw_rad_s')
    for key in positive:
        value = float(control[key])
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f'control.{key} must be finite and positive')
        control[key] = value
    if control['ready_timeout_s'] <= control['ready_hold_s']:
        raise ValueError('ready_timeout_s must exceed ready_hold_s')
    if not 0.002 <= control['motor_dt'] <= 0.02:
        raise ValueError('motor_dt must be between 0.002 and 0.02 seconds')
    if not 0 <= control['joystick_deadzone'] < 1 or not 0 < control['min_depth_valid_fraction'] <= 1:
        raise ValueError('Invalid deadzone/depth validity fraction')
    if cfg['expected_mode_machine'] not in (1, 4):
        raise ValueError('Only native 23DoF mode_machine 1 or 4 supported; confirm hardware revision')
    if control['max_tilt_rad'] >= np.pi/2:
        raise ValueError('max_tilt_rad must be less than pi/2')
    if not 0 < cfg['camera']['stream_timeout_s'] <= control['depth_timeout_s']:
        raise ValueError('Camera timeout must not exceed depth watchdog')
    return cfg
