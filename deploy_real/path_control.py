"""External localization input. No position is synthesized from IMU acceleration."""
from __future__ import annotations

import json
from pathlib import Path
import time
from common.path_controller import PathController, PathSettings, PoseSample, PoseUnavailableError


class JsonPoseSource:
    """Read an atomically replaced local JSON snapshot stamped with monotonic_ns."""
    def __init__(self, path):
        self.path = Path(path).expanduser()

    def snapshot(self):
        try:
            data = json.loads(self.path.read_text())
            if not isinstance(data['valid'], bool):
                raise ValueError('valid must be a boolean')
            return PoseSample(timestamp_s=int(data['monotonic_ns'])*1e-9,
                              xy=tuple(data['xy_m']), yaw=float(data['yaw_rad']),
                              frame_id=data['frame_id'], valid=data['valid'],
                              confidence=float(data['confidence']))
        except (OSError, ValueError, TypeError, KeyError, OverflowError) as exc:
            raise PoseUnavailableError(f'Cannot read localization snapshot: {exc}') from exc


class RealPathControl:
    def __init__(self, cfg, control, pose_source=None):
        settings = PathSettings.from_dict(cfg)
        if settings.enabled and (settings.max_forward_m_s > control['max_forward_m_s'] or
                                 settings.max_yaw_rad_s > control['max_yaw_rad_s']):
            raise ValueError('Path limits must not exceed real control velocity limits')
        self.controller = PathController(settings)
        self.max_skew_s = float((cfg or {}).get('max_state_skew_s', .05))
        if not 0 < self.max_skew_s <= settings.state_timeout_s:
            raise ValueError('max_state_skew_s must be positive and <= state_timeout_s')
        self.source = pose_source
        if settings.enabled and self.source is None:
            source_cfg = (cfg or {}).get('pose_source') or {}
            if source_cfg.get('type') != 'json_file' or not source_cfg.get('path'):
                raise ValueError('Enable path control only with a configured localization source')
            self.source = JsonPoseSource(source_cfg['path'])

    def reset(self):
        self.controller.reset()

    def update(self, state, requested, now_s=None):
        if not self.controller.cfg.enabled:
            return self.controller.update(time.monotonic() if now_s is None else now_s, None, requested)
        # Stop does not require localization, but must not establish a new path.
        if requested[0] == 0 and requested[2] == 0:
            return self.controller.update(time.monotonic() if now_s is None else now_s, None, requested)
        try:
            pose = self.source.snapshot()
            if abs(pose.timestamp_s-state.received_at_ns*1e-9) > self.max_skew_s:
                raise PoseUnavailableError('Localization and LowState timestamps are not aligned')
            # Sample control time after I/O so a newly published pose is not
            # incorrectly classified as being in the future.
            return self.controller.update(time.monotonic() if now_s is None else now_s, pose, requested)
        except PoseUnavailableError as exc:
            self.controller.diagnostics = {'state': 'INVALID', 'error': str(exc)}
            raise
