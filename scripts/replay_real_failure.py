"""Inspect a saved real-deployment fault and replay its ONNX inputs offline."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from deploy_real.parkour_policy import ParkourPolicy
from deploy_mujoco.deploy_mujoco_env import _run_depth_encoder_actor_onnx


def inspect_report(path):
    report=json.loads(Path(path).read_text())
    print('Failure:',report.get('error','not recorded'))
    tracking=report.get('tracking_error')
    if tracking:
        print(f"Phase: {tracking['phase']}; elapsed={tracking['phase_elapsed_s']:.3f}s; "
              f"limit={tracking['limit_rad']:.3f} rad; rejected candidate sent={tracking['candidate_sent']}")
        print('joint                         motor  measured  previous  candidate   error (rad)')
        for joint in tracking['joints'][:5]:
            print(f"{joint['name']:30} {joint['motor_slot']:3} "
                  f"{joint['measured_q_rad']:9.4f} {joint['previous_target_rad']:9.4f} "
                  f"{joint['candidate_target_rad']:9.4f} {joint['error_rad']:+9.4f}")
    inputs=report.get('last_inference') or {}
    if inputs.get('proprio') is None or inputs.get('depth_history') is None or not tracking:
        print('No matching tracking snapshot and ONNX inputs; exact inference replay unavailable.')
        return
    if tracking['action_generation'] != tracking['controller_generation']:
        print('Action generation did not match controller; skip replay comparison.')
        return
    policy=ParkourPolicy(report['config']['policy_config'])
    proprio=np.asarray(inputs['proprio'],np.float32)
    depth=np.asarray(inputs['depth_history'],np.float32)
    if proprio.shape!=(624,) or depth.shape!=(8,18,32) or not np.isfinite(proprio).all() or not np.isfinite(depth).all():
        raise ValueError('Invalid saved ONNX inputs')
    replay=_run_depth_encoder_actor_onnx(policy.encoder,policy.actor,proprio,depth)
    saved=np.empty(23,np.float32)
    for joint in tracking['joints']:
        saved[joint['policy_index']]=joint['raw_action']
    error=float(np.max(np.abs(replay-saved)))
    np.testing.assert_allclose(replay,saved,rtol=1e-5,atol=1e-5)
    print(f'ONNX replay PASS; maximum raw-action difference={error:.8g}')
    print('This reproduces inference only, not physical motor response.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report',type=Path)
    inspect_report(parser.parse_args().report)


if __name__=='__main__':main()
