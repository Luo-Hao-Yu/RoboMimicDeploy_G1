"""Bounded tracking recovery; never relaxes the transmitted error/rate bounds."""
from __future__ import annotations

import numpy as np


def tracking_recovery_config(control):
    soft = float(control['max_position_error_rad'])
    cfg = dict(enabled=False, hard_error_rad=soft*1.25, release_error_rad=soft*.8125,
               timeout_s=.5, release_hold_s=.1)
    supplied = control.get('tracking_recovery', {})
    if not isinstance(supplied, dict):
        raise ValueError('control.tracking_recovery must be a mapping')
    unknown = set(supplied)-set(cfg)
    if unknown:
        raise ValueError(f'Unknown tracking_recovery settings: {sorted(unknown)}')
    cfg.update(supplied)
    if type(cfg['enabled']) is not bool:
        raise ValueError('tracking_recovery.enabled must be boolean')
    for name in ('hard_error_rad', 'release_error_rad', 'timeout_s', 'release_hold_s'):
        cfg[name] = float(cfg[name])
        if not np.isfinite(cfg[name]) or cfg[name] <= 0:
            raise ValueError(f'tracking_recovery.{name} must be finite and positive')
    if not 0 < cfg['release_error_rad'] < soft < cfg['hard_error_rad']:
        raise ValueError('Require release_error_rad < max_position_error_rad < hard_error_rad')
    if not control['motor_dt'] <= cfg['release_hold_s'] < cfg['timeout_s']:
        raise ValueError('Require motor_dt <= release_hold_s < tracking recovery timeout_s')
    return cfg


class TrackingGuardError(ValueError):
    def __init__(self, reason, limit_rad):
        self.reason, self.limit_rad = reason, limit_rad
        super().__init__(reason)


class TrackingGuard:
    def __init__(self, control):
        self.cfg = tracking_recovery_config(control)
        self.soft = float(control['max_position_error_rad'])
        self.state_timeout_s = float(control['state_timeout_s'])
        self.reset()

    def reset(self):
        self.active = False
        self.started_ns = None
        self.healthy_since_ns = None
        self.last_sample_ns = None
        self.last_now_ns = None
        self.watched = None
        self.episodes = self.recoveries = self.limited_cycles = 0
        self.diagnostics = dict(status='normal' if self.cfg['enabled'] else 'disabled', active=False)

    def statistics(self):
        return dict(self.diagnostics, episodes=self.episodes, recoveries=self.recoveries,
                    limited_cycles=self.limited_cycles)

    def _fault(self, reason, limit=None):
        self.diagnostics.update(status='fault', reason=reason, active=self.active)
        raise TrackingGuardError(reason, self.soft if limit is None else limit)

    def apply(self, desired, candidate, previous, measured, position_limits, step, now_ns, sample_ns):
        """Project into the intersection, then monitor original demand for recovery.

        desired is position-limited but not rate-limited; candidate is the usual
        rate-limited target. Monitoring desired prevents clipping from falsely
        reporting that a persistently blocked joint has recovered.
        """
        error = np.abs(candidate-measured)
        max_error = float(error.max())
        over = error > self.soft
        self.diagnostics = dict(status='normal', active=self.active,
                                candidate_max_error_rad=max_error,
                                soft_limit_rad=self.soft, clipped_indices=[])
        if self.last_now_ns is not None and now_ns < self.last_now_ns:
            self._fault('control_time_regressed')
        self.last_now_ns = now_ns
        if max_error > self.cfg['hard_error_rad']:
            self._fault('hard_tracking_error', self.cfg['hard_error_rad'])
        if not self.active and not over.any():
            return candidate
        age = (now_ns-sample_ns)*1e-9
        if not 0 <= age <= self.state_timeout_s:
            self._fault('invalid_recovery_state_age')
        if self.last_sample_ns is not None and sample_ns < self.last_sample_ns:
            self._fault('recovery_state_time_regressed')
        fresh = self.last_sample_ns is None or sample_ns > self.last_sample_ns
        sample_gap = self.last_sample_ns is not None and (sample_ns-self.last_sample_ns)*1e-9 > self.state_timeout_s
        self.last_sample_ns = sample_ns
        if not self.active:
            self.active = True
            self.started_ns = now_ns
            self.healthy_since_ns = None
            self.watched = over.copy()
            self.episodes += 1
        self.watched |= over
        elapsed = (now_ns-self.started_ns)*1e-9
        self.diagnostics.update(active=True, elapsed_s=elapsed,
                                watched_indices=np.flatnonzero(self.watched).tolist())

        # Never jump the target to the measured position when rate and tracking
        # constraints disagree. No feasible intersection means recovery is unsafe.
        lower = np.maximum.reduce([position_limits[:, 0], previous-step, measured-self.soft])
        upper = np.minimum.reduce([position_limits[:, 1], previous+step, measured+self.soft])
        if np.any(lower > upper):
            self._fault('infeasible_tracking_rate_position_bounds')
        if elapsed >= self.cfg['timeout_s']:
            self._fault('tracking_recovery_timeout')
        safe = np.clip(candidate, lower, upper)
        clipped = np.flatnonzero(np.abs(safe-candidate) > 1e-10).tolist()
        if clipped:
            self.limited_cycles += 1
        demand_error = np.maximum(np.abs(desired-measured), error)
        healthy = bool(np.all(demand_error[self.watched] <= self.cfg['release_error_rad']))
        if not healthy or sample_gap:
            self.healthy_since_ns = None
        if healthy and fresh:
            if self.healthy_since_ns is None:
                self.healthy_since_ns = sample_ns
            elif (sample_ns-self.healthy_since_ns)*1e-9 >= self.cfg['release_hold_s']:
                self.active = False
                self.recoveries += 1
        self.diagnostics.update(status=('recovered' if not self.active else 'limited' if clipped else 'recovering'),
                                active=self.active, clipped_indices=clipped,
                                sent_max_error_rad=float(np.abs(safe-measured).max()),
                                demand_max_error_rad=float(demand_error[self.watched].max()))
        return safe
