"""Offscreen validation on the supplied 23DoF plank course (EGL by default)."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import time

os.environ.setdefault('MUJOCO_GL', 'egl')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import mujoco
import numpy as np
from deploy_mujoco.deploy_mujoco_env import (
    _deploy_env_cfg_for_interface, _make_policy_runtime, _setup_simulation,
    load_deploy_config, pd_control, policy_dir_from_deploy_yaml, resolve_path,
)


def validate(seconds=10.0, vx=0.0, warmup=2.0, report=None):
    path, cfg = load_deploy_config('config/g1_23dof/parkour/params/deploy.yaml', ROOT)
    cfg['policy_interface']['preview_depth'] = False
    if seconds <= 0 or warmup < 0 or not np.isfinite([seconds, vx, warmup]).all():
        raise ValueError('Expected positive seconds, nonnegative warmup, and finite arguments')
    limits = cfg['commands']['base_velocity']['ranges']['lin_vel_x']
    if not limits[0] <= vx <= limits[1]:
        raise ValueError(f'vx must be within {limits}')
    command = np.zeros(3, dtype=np.float32)
    model, data, env, maps, kp, kd = _setup_simulation(
        _deploy_env_cfg_for_interface(cfg, 'depth_encoder_actor'),
        resolve_path(cfg['xml_path'], ROOT), cfg['simulation_dt'], None, command,
    )
    runtime = _make_policy_runtime('depth_encoder_actor', cfg, policy_dir_from_deploy_yaml(path), env, data, maps, command, 'cpu')
    target = env.target_sdk_joint_pos()
    min_z = float(data.qpos[2])
    max_tilt = 0.0
    max_action = 0.0
    trajectory = []
    fallen = False
    terrain = json.loads((ROOT / 'g1_description/g1_23dof_parkour/terrain_manifest.json').read_text())['planks']
    max_cross_track = 0.0
    first_off_course_time = None
    start = time.perf_counter()
    decimation = round(env.step_dt / model.opt.timestep)
    total_steps = round(seconds / model.opt.timestep)
    count = 0
    try:
        for count in range(1, total_steps + 1):
            data.ctrl[maps.actuator] = pd_control(target, data.qpos[maps.qposadr], kp,
                                                 np.zeros(env.num_joints), data.qvel[maps.qveladr], kd)
            mujoco.mj_step(model, data)
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or any(w.number for w in data.warning):
                raise FloatingPointError('Invalid MuJoCo state or physics warning')
            if count % decimation == 0:
                command[0] = vx if data.time >= warmup else 0
                env.sync_robot_from_mujoco(data.qpos, data.qvel, maps.qposadr, maps.qveladr)
                action = runtime.act()
                assert action.shape == (23,) and np.isfinite(action).all()
                env.process_action(action)
                target = env.target_sdk_joint_pos()
                env.episode_length += 1
                max_action = max(max_action, float(np.abs(action).max()))
                z = float(data.qpos[2])
                tilt = float(np.degrees(np.arccos(np.clip(data.xmat[model.body('pelvis').id].reshape(3, 3)[2, 2], -1, 1))))
                min_z, max_tilt = min(min_z, z), max(max_tilt, tilt)
                cross_track = abs(float(data.qpos[1]) - terrain['start'][1])
                max_cross_track = max(max_cross_track, cross_track)
                if cross_track > terrain['plank_length'] / 2 and first_off_course_time is None:
                    first_off_course_time = float(data.time)
                if env.episode_length % 50 == 0:
                    point = {'time': round(data.time, 3), 'position': data.qpos[:3].tolist(), 'tilt_deg': tilt}
                    trajectory.append(point)
                    print(json.dumps(point), flush=True)
                if z < 0.5 or tilt > 60:
                    fallen = True
                    break
        depth = env.compute_sensor('depth')
        assert depth.shape == (18, 32) and np.isfinite(depth).all()
        result = {'requested_seconds': seconds, 'simulated_seconds': float(data.time),
                  'wall_seconds': time.perf_counter() - start, 'vx': vx, 'warmup_seconds': warmup,
                  'fallen': fallen, 'off_course': first_off_course_time is not None,
                  'first_off_course_time': first_off_course_time, 'max_cross_track_m': max_cross_track,
                  'min_pelvis_z': min_z, 'max_tilt_deg': max_tilt,
                  'max_abs_action': max_action, 'final_position': data.qpos[:3].tolist(),
                  'depth_range': [float(depth.min()), float(depth.max())],
                  'trajectory': trajectory}
        if report:
            output = Path(report)
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps({k: v for k, v in result.items() if k != 'trajectory'}, indent=2), flush=True)
        return result
    finally:
        for renderer in env._depth_renderers.values():
            renderer.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--vx', type=float, default=0)
    parser.add_argument('--warmup', type=float, default=2)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    result = validate(args.seconds, args.vx, args.warmup, args.report)
    raise SystemExit(1 if result['fallen'] or result['off_course'] else 0)
