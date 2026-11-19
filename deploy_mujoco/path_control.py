"""MuJoCo pelvis truth adapter; operator intent never aliases final commands."""
from __future__ import annotations

import mujoco
from common.path_controller import PathController, PathSettings, PoseSample, yaw_wxyz


class SimulationPathControl:
    def __init__(self, cfg, model, data):
        self.model, self.data = model, data
        self.controller = PathController(PathSettings.from_dict(cfg))
        pelvis = model.body('pelvis').id
        joint = int(model.body_jntadr[pelvis])
        if joint < 0 or model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError('Path control requires a free pelvis root joint')
        self.root_adr = int(model.jnt_qposadr[joint])
        self.reset()

    def pose(self):
        q = self.data.qpos[self.root_adr:self.root_adr+7]
        return PoseSample(float(self.data.time), tuple(q[:2]), yaw_wxyz(q[3:7]),
                          frame_id=self.controller.cfg.frame_id)

    def reset(self):
        self.controller.reset(self.pose())

    def update(self, requested):
        return self.controller.update(float(self.data.time), self.pose(), requested)
