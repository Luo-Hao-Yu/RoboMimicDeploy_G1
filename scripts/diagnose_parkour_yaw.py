"""Compare straight-line Parkour tracking without changing deployment settings."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

os.environ.setdefault("MUJOCO_GL", "egl")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from deploy_mujoco import deploy_mujoco_env as api

CASES = {
    "standing": {"vx": 0.0},
    "forward": {},
    "flat": {"flat": True},
    "half_timestep": {"dt": 0.0025},
    "heading_hold": {"feedback": "heading"},
    "cross_track": {"feedback": "cross_track"},
}


def run_case(name, seconds, vx, warmup=2.0, *, flat=False, dt=0.005, feedback=None):
    path, cfg = api.load_deploy_config(api.DEFAULT_CONFIG, ROOT)
    cfg["policy_interface"]["preview_depth"] = False
    command = np.zeros(3, np.float32)
    model, data, env, maps, kp, kd = api._setup_simulation(
        api._deploy_env_cfg_for_interface(cfg, "depth_encoder_actor"),
        api.resolve_path(cfg["xml_path"], ROOT), dt, None, command,
    )
    if flat:
        # This changes both contacts and depth images; it only tests whether
        # plank edges/gaps are necessary for the yaw drift.
        for gid in range(model.ngeom):
            if model.geom_bodyid[gid] == 0 and model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_BOX:
                model.geom_contype[gid] = 0
                model.geom_conaffinity[gid] = 0
                model.geom_pos[gid, 2] = -10
        model.geom_pos[model.geom("floor").id, 2] = 0
        mujoco.mj_forward(model, data)

    samples, yaw_rates = [], []
    max_cross_track = 0.0
    first_off_course = None
    saturation = np.zeros(env.num_joints)
    half_width = json.loads((ROOT / "g1_description/g1_23dof_parkour/terrain_manifest.json").read_text())["planks"]["plank_length"] / 2
    fallen = False
    count = 0
    try:
        runtime = api._make_policy_runtime(
            "depth_encoder_actor", cfg, api.policy_dir_from_deploy_yaml(path),
            env, data, maps, command, "cpu",
        )
        target = env.target_sdk_joint_pos()
        for count in range(1, round(seconds / dt) + 1):
            torque = api.pd_control(target, data.qpos[maps.qposadr], kp,
                                    np.zeros(env.num_joints), data.qvel[maps.qveladr], kd)
            saturation += np.abs(torque) > model.actuator_ctrlrange[maps.actuator, 1]
            data.ctrl[maps.actuator] = torque
            mujoco.mj_step(model, data)
            if not np.isfinite(data.qpos).all() or any(w.number for w in data.warning):
                raise FloatingPointError("Invalid physics state or MuJoCo warning")
            if count % round(env.step_dt / dt):
                continue

            rotation = Rotation.from_quat(np.roll(data.qpos[3:7], -1))
            yaw = float(rotation.as_euler("xyz")[2])
            command[:] = [vx, 0, 0] if data.time >= warmup else 0
            if feedback and data.time >= warmup:
                # Diagnostic controllers only; production behavior is unchanged.
                target_heading = np.arctan2(-data.qpos[1], 0.8) if feedback == "cross_track" else 0.0
                error = (target_heading - yaw + np.pi) % (2 * np.pi) - np.pi
                command[2] = np.clip(2 * error, -1, 1)
            env.sync_robot_from_mujoco(data.qpos, data.qvel, maps.qposadr, maps.qveladr)
            action = runtime.act()
            env.process_action(action)
            runtime.after_action(action)
            target = env.target_sdk_joint_pos()
            env.episode_length += 1

            term = next(term for term in env.obs_terms if term.name == "velocity_commands")
            np.testing.assert_array_equal(term.get_flat().reshape(8, 3)[-1], command)
            world_wz = float(rotation.apply(data.qvel[3:6])[2])
            if warmup + 1 <= data.time <= 6:
                yaw_rates.append(world_wz)
            cross_track = abs(float(data.qpos[1]))
            max_cross_track = max(max_cross_track, cross_track)
            if cross_track > half_width and first_off_course is None:
                first_off_course = float(data.time)
            sample = {
                "time": round(float(data.time), 3), "position": data.qpos[:3].tolist(),
                "yaw_deg": float(np.degrees(yaw)), "world_wz": world_wz,
                "body_velocity": rotation.inv().apply(data.qvel[:3]).tolist(),
                "command": command.tolist(),
            }
            fallen = data.qpos[2] < 0.5 or rotation.as_matrix()[2, 2] < 0.5
            if env.episode_length % 50 == 0 or fallen:
                samples.append(sample)
            if fallen:
                break

        result = {
            "case": name, "vx": vx, "dt": dt, "flat": flat, "feedback": feedback,
            "simulated_seconds": float(data.time), "fallen": bool(fallen),
            "final": sample, "max_cross_track_m": max_cross_track,
            "first_off_course_time": first_off_course,
            "mean_wz_3_to_6s": float(np.mean(yaw_rates)) if yaw_rates else None,
            "torque_saturation_fraction": dict(zip(env.sdk_joint_names, (saturation / count).tolist())),
            "samples": samples,
        }
        print(json.dumps({key: value for key, value in result.items()
                          if key not in ("samples", "torque_saturation_fraction")}), flush=True)
        return result
    finally:
        for renderer in env._depth_renderers.values():
            renderer.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", nargs="+", choices=CASES, default=list(CASES))
    parser.add_argument("--seconds", type=float, default=15)
    parser.add_argument("--vx", type=float, default=0.4)
    parser.add_argument("--report", type=Path, default=ROOT / "logs/g1_23dof/yaw_diagnosis.json")
    args = parser.parse_args()
    if not np.isfinite([args.seconds, args.vx]).all() or args.seconds < 0.02 or not 0 <= args.vx <= 0.65:
        parser.error("seconds must be >=0.02; vx must be in [0, 0.65]; both must be finite")
    results = []
    for name in args.cases:
        options = {"vx": args.vx, **CASES[name]}
        results.append(run_case(name, args.seconds, **options))
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(results, indent=2) + "\n")


if __name__ == "__main__":
    main()
