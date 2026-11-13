"""Shared planar path feedback. Coordinates: forward X, left Y, CCW yaw."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import numpy as np


def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def yaw_wxyz(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or not .9 < np.linalg.norm(q) < 1.1:
        raise ValueError('Expected a finite normalized pelvis quaternion in wxyz order')
    w, x, y, z = q / np.linalg.norm(q)
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


@dataclass(frozen=True)
class PoseSample:
    timestamp_s: float
    xy: tuple[float, float] | None
    yaw: float
    frame_id: str = 'local'
    valid: bool = True
    confidence: float = 1.0


class PathState(str, Enum):
    STOP = 'STOP'
    PATH_FOLLOW = 'PATH_FOLLOW'
    MANUAL_TURN = 'MANUAL_TURN'


class PoseUnavailableError(ValueError):
    pass


@dataclass(frozen=True)
class PathSettings:
    enabled: bool = False
    mode: str = 'path'
    reference: str = 'fixed'
    origin_xy: tuple[float, float] = (0., 0.)
    yaw_rad: float = 0.
    frame_id: str = 'local'
    heading_gain: float = 2.
    lookahead_m: float = 1.
    max_yaw_rad_s: float = 1.
    max_forward_m_s: float = .65
    state_timeout_s: float = .1
    min_confidence: float = .8
    manual_rebase: bool = False

    def __post_init__(self):
        if self.mode not in ('path', 'heading', 'constant'):
            raise ValueError('path_control.mode must be path, heading or constant')
        if self.reference not in ('fixed', 'startup', 'current'):
            raise ValueError('path_control.reference must be fixed, startup or current')
        if self.manual_rebase and self.reference != 'startup':
            raise ValueError('manual_rebase is only allowed for an open-ground startup path')
        for name in ('heading_gain', 'lookahead_m', 'max_yaw_rad_s', 'max_forward_m_s', 'state_timeout_s'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'path_control.{name} must be finite and positive')
        if (np.asarray(self.origin_xy).shape != (2,) or
                not np.isfinite([*self.origin_xy, self.yaw_rad]).all()):
            raise ValueError('Expected finite path origin_xy and yaw_rad')
        if not self.frame_id or not 0 <= self.min_confidence <= 1:
            raise ValueError('Expected frame_id and min_confidence in [0,1]')
        for name in ('enabled', 'manual_rebase'):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(f'path_control.{name} must be boolean')

    @classmethod
    def from_dict(cls, cfg=None):
        cfg = cfg or {}
        return cls(**{name: cfg[name] for name in cls.__dataclass_fields__ if name in cfg})


class PathController:
    def __init__(self, settings):
        self.cfg = settings
        self.reset()

    def reset(self, pose=None):
        self.state = PathState.STOP
        self.origin = None
        self.path_yaw = None
        self.last_update_s = None
        self.last_pose_s = None
        self.pending_rebase = False
        self.diagnostics = {'state': self.state.value}
        if self.cfg.reference == 'fixed':
            self.origin = np.array(self.cfg.origin_xy, dtype=float)
            self.path_yaw = self.cfg.yaw_rad
        if pose is not None:
            self._check_pose(pose, pose.timestamp_s)
            self._anchor_if_needed(pose)

    def _check_pose(self, pose, now):
        if pose is None:
            raise PoseUnavailableError('Feedback requires a valid pelvis orientation sample')
        if (not pose.valid or not math.isfinite(pose.confidence) or
                not self.cfg.min_confidence <= pose.confidence <= 1 or
                not np.isfinite([pose.timestamp_s, pose.yaw]).all()):
            raise PoseUnavailableError('Invalid or low-confidence path pose')
        if self.cfg.mode == 'path' and pose.xy is None:
            raise PoseUnavailableError('Path mode requires pelvis XY; IMU heading alone is insufficient')
        if pose.xy is not None and (np.asarray(pose.xy).shape != (2,) or not np.isfinite(pose.xy).all()):
            raise PoseUnavailableError('Invalid pelvis XY')
        if pose.frame_id != self.cfg.frame_id:
            raise PoseUnavailableError('Pose frame changed: transform the path and explicitly reset')
        if not -1e-6 <= now - pose.timestamp_s <= self.cfg.state_timeout_s:
            raise PoseUnavailableError('Stale or future path pose')
        if self.last_pose_s is not None and pose.timestamp_s < self.last_pose_s:
            raise PoseUnavailableError('Pose timestamp moved backwards: explicitly reset')
        self.last_pose_s = pose.timestamp_s

    def _anchor_if_needed(self, pose):
        if self.path_yaw is None:
            self._set_reference(pose)

    def _set_reference(self, pose):
        self.origin = None if pose.xy is None else np.array(pose.xy, dtype=float)
        self.path_yaw = pose.yaw

    def update(self, now_s, pose, requested):
        requested = np.asarray(requested, dtype=float)
        if requested.shape != (3,) or not np.isfinite(requested).all() or not math.isfinite(now_s):
            raise ValueError('Expected finite time and [vx,vy,wz] intent')
        if not self.cfg.enabled:
            self.diagnostics = {'state': 'DISABLED', 'requested': requested.tolist()}
            return requested.astype(np.float32)
        if self.last_update_s is not None and now_s < self.last_update_s:
            raise PoseUnavailableError('Control time moved backwards: explicitly reset')
        self.last_update_s = now_s
        vx = float(np.clip(requested[0], 0, self.cfg.max_forward_m_s))
        manual_wz = float(np.clip(requested[2], -self.cfg.max_yaw_rad_s, self.cfg.max_yaw_rad_s))
        self.diagnostics = {'state': self.state.value, 'requested': requested.tolist()}
        # A released stick is an exact stop, regardless of position/yaw error.
        if vx == 0 and manual_wz == 0:
            self.state = PathState.STOP
            self.diagnostics['state'] = self.state.value
            return np.zeros(3, np.float32)
        try:
            self._check_pose(pose, now_s)
        except PoseUnavailableError as exc:
            self.diagnostics.update(state='INVALID', error=str(exc))
            raise
        self._anchor_if_needed(pose)
        # Latch once per straight segment, not every frame: otherwise both
        # heading and cross-track feedback would lose their reference.
        if (self.cfg.reference == 'current' and manual_wz == 0 and
                self.state != PathState.PATH_FOLLOW):
            self._set_reference(pose)
        normal = np.array([-math.sin(self.path_yaw), math.cos(self.path_yaw)])
        cross_track = (float(normal @ (np.asarray(pose.xy) - self.origin))
                       if pose.xy is not None and self.origin is not None else None)
        desired = self.path_yaw
        saturated = False
        if manual_wz != 0:
            self.state = PathState.MANUAL_TURN
            self.pending_rebase = True
            wz = manual_wz
        else:
            if self.pending_rebase and self.cfg.manual_rebase:
                self._set_reference(pose)
                cross_track = 0. if pose.xy is not None else None
                desired = self.path_yaw
            self.pending_rebase = False
            self.state = PathState.PATH_FOLLOW
            if self.cfg.mode == 'path':
                desired -= math.atan2(cross_track, self.cfg.lookahead_m)
            raw_wz = self.cfg.heading_gain * wrap_angle(desired - pose.yaw)
            if self.cfg.mode == 'constant':
                raw_wz = 0.
            wz = float(np.clip(raw_wz, -self.cfg.max_yaw_rad_s, self.cfg.max_yaw_rad_s))
            saturated = abs(raw_wz) > self.cfg.max_yaw_rad_s
        command = np.array([vx, 0., wz], np.float32)
        self.diagnostics.update(state=self.state.value, cross_track_m=cross_track,
                                desired_yaw_rad=wrap_angle(desired), yaw_rad=pose.yaw,
                                heading_error_rad=wrap_angle(desired-pose.yaw), mode=self.cfg.mode,
                                pose_age_s=max(0., now_s-pose.timestamp_s),
                                origin_xy=self.origin.tolist() if self.origin is not None else None,
                                path_yaw_rad=self.path_yaw,
                                saturated=saturated, command=command.tolist())
        return command
