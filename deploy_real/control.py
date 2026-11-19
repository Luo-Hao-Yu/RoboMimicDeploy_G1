"""Pure control state machine and watchdogs, independent of DDS and camera I/O."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import numpy as np
from deploy_mujoco.gamepad_control import axis_velocity
from deploy_real.tracking_guard import TrackingGuard, TrackingGuardError

START, SELECT, F1, A = (1 << n for n in (2, 3, 6, 8))


class Phase(str, Enum):
    WAITING = 'waiting'
    RAMP = 'ramp'
    SETTLING = 'settling'
    READY = 'ready'
    POLICY = 'policy'
    FAULT = 'fault'


@dataclass
class ActionSample:
    target: np.ndarray
    raw_action: np.ndarray
    created_ns: int
    state_ns: int
    depth_ns: int
    generation: int
    command: np.ndarray | None = None
    proprio: np.ndarray | None = None
    depth_history: np.ndarray | None = None
    requested_command: np.ndarray | None = None
    path_diagnostics: dict | None = None


class JointTrackingError(ValueError):
    """Rejected candidate target; details are captured before damping/cleanup."""
    def __init__(self, details):
        self.details = details
        worst = details['joints'][0]
        reason = details.get('reason')
        comparison = (f"error={worst['error_rad']:+.4f} rad; reason={reason}; " if reason else
                      f"error={worst['error_rad']:+.4f} rad > {details['limit_rad']:.4f} rad; ")
        super().__init__(
            f"Target/measured joint tracking protection: {worst['name']} "
            f"(motor {worst['motor_slot']}), target={worst['candidate_target_rad']:.4f} rad, "
            f"measured={worst['measured_q_rad']:.4f} rad, "
            f"{comparison}"
            f"phase={details['phase']}, elapsed={details['phase_elapsed_s']:.3f}s. "
            "Candidate was not sent."
        )


class PostureNotReadyError(ValueError):
    def __init__(self, details):
        self.details = details
        reasons = ', '.join(details.get('blocking_checks', [])) or 'stable_hold_incomplete'
        super().__init__(
            f"Posture not ready: blocking={reasons}; "
            f"largest position error: {details['worst_joint']} (motor {details['worst_motor_slot']}), "
            f"q_error={details['max_q_error_rad']:.4f} rad "
            f"(limit {details['q_tolerance_rad']:.4f}), "
            f"max_dq={details['max_dq_rad_s']:.4f} rad/s "
            f"at {details.get('fastest_joint', 'unknown')} "
            f"(limit {details['max_dq_limit_rad_s']:.4f}), "
            f"gyro_norm={details['gyro_norm_rad_s']:.4f} rad/s "
            f"(limit {details['gyro_limit_rad_s']:.4f}); "
            "posture settling failed; policy not enabled"
        )


def validate_state(state, now_ns, cfg, policy, expected_mode):
    if state is None:
        raise ValueError('No LowState received')
    age = (now_ns - state.received_at_ns) * 1e-9
    if not 0 <= age <= cfg['state_timeout_s']:
        raise ValueError(f'LowState timeout: {age:.3f}s')
    if state.mode_machine != expected_mode:
        raise ValueError(f'Robot mode_machine={state.mode_machine}, expected {expected_mode}')
    for values in (state.q, state.dq, state.gyro, state.quaternion_wxyz, state.temperature_c, state.sticks):
        if not np.isfinite(values).all():
            raise ValueError('Non-finite robot state')
    norm = np.linalg.norm(state.quaternion_wxyz)
    if not 0.9 < norm < 1.1:
        raise ValueError('Invalid IMU quaternion')
    w, x, y, z = state.quaternion_wxyz / norm
    tilt = np.arccos(np.clip(1 - 2 * (x*x + y*y), -1, 1))
    if tilt > cfg['max_tilt_rad']:
        raise ValueError(f'Robot tilt {tilt:.3f} rad exceeds limit')
    if np.any(state.temperature_c >= cfg['max_temperature_c']):
        raise ValueError('Motor overtemperature')
    if np.any(state.motor_errors):
        raise ValueError(f'Motor fault flags: {state.motor_errors.tolist()}')
    if np.any(np.abs(state.dq) > policy.velocity_limits * 1.2):
        raise ValueError('Joint velocity exceeds configured limits')
    if np.any(state.q < policy.position_limits[:, 0] - .2) or np.any(state.q > policy.position_limits[:, 1] + .2):
        raise ValueError('Measured joint position outside limits')
    if np.any(np.abs(state.sticks) > 1.1):
        raise ValueError('Invalid remote stick values')


def velocity_command(state, cfg):
    """Direct remote velocity mapping; no heading or cross-track feedback."""
    lx, ly, rx, ry = state.sticks
    deadzone = cfg['joystick_deadzone']
    return np.array([axis_velocity(ly, [0, cfg['max_forward_m_s']], deadzone),
                     0.0, axis_velocity(-rx, [-cfg['max_yaw_rad_s'], cfg['max_yaw_rad_s']], deadzone)], np.float32)


class Controller:
    def __init__(self, policy, cfg):
        self.policy, self.cfg = policy, cfg
        self.phase = Phase.WAITING
        self.generation = 0
        self.previous_buttons = None
        self.last_target = None
        self.last_time_ns = None
        self.phase_start_ns = None
        self.failure = None
        self.settling_start_ns = None
        self.ready_since_ns = None
        self.ready_last_sample_ns = None
        self.tracking_guard = TrackingGuard(cfg)

    def buttons(self, state):
        # A held button on connection cannot cause an unintentional transition.
        rising = 0 if self.previous_buttons is None else state.buttons & ~self.previous_buttons
        self.previous_buttons = state.buttons
        return rising

    def begin_ramp(self, state, now_ns):
        if self.phase != Phase.WAITING:
            raise ValueError('Cannot begin ramp from this phase')
        if np.any(np.abs(state.sticks) > self.cfg['joystick_deadzone']):
            raise ValueError('Center remote sticks before Start')
        self.initial_q = state.q.copy()
        self.last_target = state.q.copy()
        self.last_time_ns = self.phase_start_ns = now_ns
        self.phase = Phase.RAMP
        self.previous_buttons = state.buttons

    def readiness(self, state):
        error = np.abs(state.q-self.policy.default_q)
        worst = int(np.argmax(error))
        q_tolerance = self.cfg.get('ready_joint_tolerance_rad', .25)
        velocity_limit = self.cfg.get('ready_max_joint_velocity_rad_s', .2)
        gyro_limit = self.cfg.get('ready_max_gyro_rad_s', .2)
        max_dq = float(np.max(np.abs(state.dq)))
        fastest = int(np.argmax(np.abs(state.dq)))
        gyro_norm = float(np.linalg.norm(state.gyro))
        blocking = []
        if not (np.isfinite(error).all() and np.isfinite(state.dq).all() and np.isfinite(state.gyro).all()):
            blocking.append('nonfinite_state')
        if float(error[worst]) > q_tolerance: blocking.append('joint_position')
        if max_dq > velocity_limit: blocking.append('joint_velocity')
        if gyro_norm > gyro_limit: blocking.append('body_angular_velocity')
        good = not blocking
        return dict(qualified=bool(good), worst_joint=self.policy.env.sdk_joint_names[worst],
                    worst_motor_slot=int(self.policy.motor_indices[worst]),
                    fastest_joint=self.policy.env.sdk_joint_names[fastest],
                    fastest_motor_slot=int(self.policy.motor_indices[fastest]),
                    blocking_checks=blocking,
                    max_q_error_rad=float(error[worst]), q_tolerance_rad=q_tolerance,
                    max_dq_rad_s=max_dq, max_dq_limit_rad_s=velocity_limit,
                    gyro_norm_rad_s=gyro_norm, gyro_limit_rad_s=gyro_limit)

    def _check_readiness(self, state, now_ns):
        details = self.readiness(state)
        # A missed/stale observation breaks the continuous qualification window.
        age = (now_ns-state.received_at_ns)*1e-9
        fresh = 0 <= age <= self.cfg['state_timeout_s']
        if not fresh: details['blocking_checks'].append('state_age')
        if not details['qualified'] or not fresh:
            self.ready_since_ns = None
            if self.phase == Phase.READY:
                self.phase = Phase.SETTLING
                self.settling_start_ns = now_ns
        elif state.received_at_ns != self.ready_last_sample_ns:
            if (self.ready_last_sample_ns is not None and
                    (state.received_at_ns < self.ready_last_sample_ns or
                     (state.received_at_ns-self.ready_last_sample_ns)*1e-9 > self.cfg['state_timeout_s'])):
                self.ready_since_ns = None
                if self.phase == Phase.READY:
                    self.phase = Phase.SETTLING
                    self.settling_start_ns = now_ns
            if self.ready_since_ns is None:
                self.ready_since_ns = now_ns
            elif (now_ns-self.ready_since_ns)*1e-9 >= self.cfg.get('ready_hold_s', .5):
                self.phase = Phase.READY
        self.ready_last_sample_ns = state.received_at_ns
        if self.phase == Phase.SETTLING and (now_ns-self.settling_start_ns)*1e-9 >= self.cfg.get('ready_timeout_s', 8.):
            if not details['blocking_checks']:
                details['blocking_checks'].append('stable_hold_incomplete')
            details.update(measured_q_rad=state.q.tolist(), measured_dq_rad_s=state.dq.tolist(),
                           target_q_rad=self.policy.default_q.tolist(),
                           quaternion_wxyz=state.quaternion_wxyz.tolist(), gyro=state.gyro.tolist(),
                           motor_modes=state.motor_modes.tolist() if state.motor_modes is not None else None,
                           tau_est_nm=state.tau_est.tolist() if state.tau_est is not None else None,
                           state_age_s=age, settling_elapsed_s=(now_ns-self.settling_start_ns)*1e-9,
                           settling_timeout_s=self.cfg.get('ready_timeout_s', 8.),
                           stable_hold_s=0. if self.ready_since_ns is None else (now_ns-self.ready_since_ns)*1e-9,
                           required_hold_s=self.cfg.get('ready_hold_s', .5))
            raise PostureNotReadyError(details)

    def begin_policy(self, state, now_ns):
        if self.phase != Phase.READY:
            raise ValueError('Policy requires Ready state')
        if not self.readiness(state)['qualified']:
            raise PostureNotReadyError(self.readiness(state))
        if np.any(np.abs(state.sticks) > self.cfg['joystick_deadzone']):
            raise ValueError('Center remote sticks before A')
        self.phase = Phase.POLICY
        self.phase_start_ns = now_ns
        self.generation += 1
        self.tracking_guard.reset()
        return self.generation

    def fault(self, reason):
        self.failure = str(reason)
        self.phase = Phase.FAULT

    def output(self, state, now_ns, action=None):
        if self.phase == Phase.FAULT:
            return np.zeros(23), np.zeros(23), np.full(23, self.cfg['damping_kd'])
        if self.phase == Phase.WAITING:
            return None
        if state.q.shape != (23,) or not np.isfinite(state.q).all():
            raise ValueError('Invalid measured joint positions')
        elapsed = (now_ns - self.phase_start_ns) * 1e-9
        kp, kd = self.policy.kp.copy(), self.policy.kd.copy()
        if self.phase == Phase.RAMP:
            alpha = min(1., elapsed / self.cfg['ramp_s'])
            target = self.initial_q * (1-alpha) + self.policy.default_q * alpha
            kp *= alpha
            if alpha >= 1:
                self.phase = Phase.SETTLING
                self.settling_start_ns = now_ns
                self.ready_since_ns = self.ready_last_sample_ns = None
        elif self.phase in (Phase.SETTLING, Phase.READY):
            self._check_readiness(state, now_ns)
            target = self.policy.default_q.copy()
        else:
            if action is None or action.generation != self.generation:
                if elapsed > self.cfg['action_timeout_s']:
                    raise ValueError('First policy action timed out')
                target = self.last_target.copy()
            else:
                for stamp, timeout, label in (
                    (action.created_ns, self.cfg['action_timeout_s'], 'action'),
                    (action.state_ns, self.cfg['state_timeout_s'], 'action state'),
                    (action.depth_ns, self.cfg['depth_timeout_s'], 'action depth'),
                ):
                    if not 0 <= (now_ns-stamp)*1e-9 <= timeout:
                        raise ValueError(f'Stale {label}')
                if (action.target.shape != (23,) or not np.isfinite(action.target).all()
                        or action.raw_action.shape != (23,) or not np.isfinite(action.raw_action).all()
                        or np.max(np.abs(action.raw_action)) > self.cfg['max_abs_action']):
                    raise ValueError('Invalid policy output')
                alpha = min(1., elapsed / self.cfg['policy_blend_s'])
                target = self.policy.default_q * (1-alpha) + action.target * alpha
        blended_target = target.copy()
        # Position/velocity protection modifies execution only; last_action is raw actor output.
        if self.phase != Phase.RAMP:
            target = np.clip(target, self.policy.position_limits[:, 0], self.policy.position_limits[:, 1])
        position_limited_target = target.copy()
        dt = max(0., min((now_ns-self.last_time_ns)*1e-9, self.cfg['motor_dt']*2))
        step = self.cfg['max_target_step_rad_s'] * dt
        target = np.clip(target, self.last_target-step, self.last_target+step)
        errors = target-state.q
        rejected_target = target.copy()
        reason = None
        fault_limit = self.cfg['max_position_error_rad']
        if self.phase == Phase.POLICY and self.tracking_guard.cfg['enabled']:
            try:
                target = self.tracking_guard.apply(position_limited_target, target, self.last_target,
                    state.q, self.policy.position_limits, step, now_ns, state.received_at_ns)
            except TrackingGuardError as exc:
                reason, fault_limit = exc.reason, exc.limit_rad
        elif np.max(np.abs(errors)) > fault_limit:
            reason = 'strict_tracking_error'
        if reason is not None:
            matching_action = action is not None and action.generation == self.generation
            policy_indices = np.argsort(self.policy.env.joint_ids_map)
            joints = []
            for i in np.argsort(-np.abs(errors)):
                j = int(i)
                joints.append(dict(
                    name=self.policy.env.sdk_joint_names[j], compact_index=j,
                    policy_index=int(policy_indices[j]), motor_slot=int(self.policy.motor_indices[j]),
                    measured_q_rad=float(state.q[j]), measured_dq_rad_s=float(state.dq[j]),
                    default_q_rad=float(self.policy.default_q[j]),
                    raw_action=float(action.raw_action[policy_indices[j]]) if matching_action else None,
                    policy_target_rad=float(action.target[j]) if matching_action else None,
                    blended_target_rad=float(blended_target[j]),
                    position_limited_target_rad=float(position_limited_target[j]),
                    candidate_target_rad=float(rejected_target[j]), previous_target_rad=float(self.last_target[j]),
                    error_rad=float(errors[j]),
                    # This is an estimate from the candidate, not measured motor torque.
                    candidate_pd_torque_estimate_nm=float(kp[j]*errors[j]-kd[j]*state.dq[j]),
                    kp=float(kp[j]), kd=float(kd[j]),
                ))
            raise JointTrackingError(dict(
                phase=self.phase.value, phase_elapsed_s=elapsed,
                policy_blend_fraction=min(1., elapsed/self.cfg['policy_blend_s']) if self.phase == Phase.POLICY else None,
                limit_rad=fault_limit, tick=int(state.tick), reason=reason,
                tracking_recovery=self.tracking_guard.statistics(),
                state_age_s=(now_ns-state.received_at_ns)*1e-9,
                action_generation=action.generation if action is not None else None,
                controller_generation=self.generation,
                action_age_s=(now_ns-action.created_ns)*1e-9 if action is not None else None,
                depth_age_s=(now_ns-action.depth_ns)*1e-9 if action is not None else None,
                quaternion_wxyz=state.quaternion_wxyz.tolist(), gyro=state.gyro.tolist(),
                sticks=state.sticks.tolist(), buttons=int(state.buttons),
                candidate_sent=False, joints=joints,
            ))
        self.last_target, self.last_time_ns = target.copy(), now_ns
        return target, kp, kd
