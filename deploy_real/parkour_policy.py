"""23DoF real-state adapter for the exact MuJoCo Parkour ONNX contract."""
from __future__ import annotations

import json
from pathlib import Path
import numpy as np
import yaml
from deploy_mujoco.deploy_mujoco_env import (
    DEPLOY_ROOT, G1_29DOF_SDK_JOINT_NAMES, DeployEnv, DepthHistoryBuffer,
    _deploy_env_cfg_for_interface, _load_depth_encoder_actor_onnx,
    _validate_export_interface, _run_depth_encoder_actor_onnx,
    policy_dir_from_deploy_yaml,
)
from .depth_source import preprocess_depth


class ParkourPolicy:
    def __init__(self, config_path):
        path = Path(config_path)
        if not path.is_absolute():
            path = DEPLOY_ROOT / path
        self.cfg = yaml.safe_load(path.read_text())
        self.model_dir = policy_dir_from_deploy_yaml(path) / 'exported'
        self.metadata = json.loads((self.model_dir / 'policy_metadata.json').read_text())
        self.env = DeployEnv(_deploy_env_cfg_for_interface(self.cfg, 'depth_encoder_actor'))
        if self.env.num_joints != 23 or self.cfg['policy_interface']['type'] != 'depth_encoder_actor':
            raise ValueError('Real deployment requires the 23DoF depth_encoder_actor policy')
        self.motor_indices = [G1_29DOF_SDK_JOINT_NAMES.index(n) for n in self.env.sdk_joint_names]
        mapped = [self.motor_indices[i] for i in self.env.joint_ids_map]
        if mapped != self.metadata['action_sdk_motor_indices']:
            raise ValueError('Exported hardware motor indices disagree with joint names')
        expected = list(range(13)) + list(range(15, 20)) + list(range(22, 27))
        if self.motor_indices != expected:
            raise ValueError('Only native G1 23DoF compact joint layout is supported')
        interface = dict(self.cfg['policy_interface'], verify_export=True, providers=['CPUExecutionProvider'])
        self.encoder, self.actor = _load_depth_encoder_actor_onnx(self.model_dir, interface)
        self.env.reset()
        self.env.compute_obs()
        _validate_export_interface(self.encoder, self.actor, self.env, self.model_dir)
        self.kp, self.kd = self.env.stiffness.copy(), self.env.damping.copy()
        self.default_q = self.to_compact(np.asarray(self.metadata['action_offset_rad']))
        self.position_limits = self.to_compact(np.asarray(self.metadata['joint_position_limits_rad']))
        self.velocity_limits = self.to_compact(np.asarray(self.metadata['velocity_limits_rad_s']))
        self.effort_limits = self.to_compact(np.asarray(self.metadata['effort_limits_nm']))
        for name, values in (('limits', self.position_limits), ('velocity', self.velocity_limits),
                             ('effort', self.effort_limits), ('default', self.default_q)):
            if not np.isfinite(values).all():
                raise ValueError(f'Non-finite metadata {name}')
        if (self.position_limits.shape != (23,2) or np.any(self.position_limits[:,0] >= self.position_limits[:,1])
                or np.any(self.velocity_limits <= 0) or np.any(self.effort_limits <= 0)):
            raise ValueError('Invalid joint limits in metadata')
        self.dt = self.env.step_dt
        self.depth_cfg = self.cfg['observations']['depth']
        self.reset()

    def to_compact(self, policy_array):
        result = np.empty_like(policy_array)
        result[self.env.joint_ids_map] = policy_array
        return result

    def reset(self):
        self.env.reset()
        shape = tuple(self.metadata['depth_encoder_input_shape'][2:])
        self.depth_history = DepthHistoryBuffer(self.depth_cfg['history_length'], shape)
        self.last_proprio = None
        self.last_depth = None

    def observe_state(self, state, command):
        if (state.q.shape != (23,) or state.dq.shape != (23,) or
                state.gyro.shape != (3,) or state.quaternion_wxyz.shape != (4,)):
            raise ValueError('Bad real robot state dimensions')
        quat = np.asarray(state.quaternion_wxyz, np.float32)
        if not np.isfinite(quat).all() or not 0.9 < np.linalg.norm(quat) < 1.1:
            raise ValueError('Invalid pelvis quaternion; expected wxyz')
        robot = self.env.robot
        robot.root_quat_wxyz = quat / np.linalg.norm(quat)
        robot.root_ang_vel_b = state.gyro.copy()
        robot.joint_pos_sdk = state.q.copy()
        robot.joint_pos_policy = state.q[self.env.joint_ids_map].copy()
        robot.joint_vel_policy = state.dq[self.env.joint_ids_map].copy()
        robot.velocity_command[:] = command

    def step(self, state, command, metric_depth):
        self.observe_state(state, command)
        self.depth_history.append(preprocess_depth(metric_depth, self.depth_cfg))
        depth = self.depth_history.delayed_frames(self.depth_cfg['num_output_frames'],
                                                self.depth_cfg['skip_frames'],
                                                self.depth_cfg.get('delay_frames', 0))
        proprio = self.env.compute_obs()['obs']
        action = _run_depth_encoder_actor_onnx(self.encoder, self.actor, proprio, depth)
        self.env.process_action(action)
        self.env.episode_length += 1
        self.last_proprio, self.last_depth = proprio.copy(), depth.copy()
        return action, self.env.target_sdk_joint_pos()
