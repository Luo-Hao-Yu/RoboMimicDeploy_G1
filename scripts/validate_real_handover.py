"""Offline grounded Ready-to-Policy experiment; never opens DDS or real cameras.

Initialize the simulation in the exported default posture, hold it with full PD
for --warmup seconds, then exercise the real Controller and policy adapter.
This is a synthetic scenario, not a replay of the user's physical robot.
"""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import mujoco,numpy as np
from deploy_real.parkour_policy import ParkourPolicy
from deploy_real.control import Controller,Phase,ActionSample,JointTrackingError,validate_state
from deploy_real.real_config import load_config
from deploy_real.robot_state import RobotState
from deploy_mujoco import deploy_mujoco_env as sim

root=Path(__file__).resolve().parents[1]

def run(warmup, blend=1.0, executed_history=False, seconds=10.0):
    cfg=load_config();cfg['control']['policy_blend_s']=blend;p=ParkourPolicy(cfg['policy_config'])
    m,d,env,maps,kp,kd=sim._setup_simulation(sim._deploy_env_cfg_for_interface(p.cfg,'depth_encoder_actor'),
        root/p.cfg['xml_path'],.005,None,np.zeros(3,np.float32))
    renderer=mujoco.Renderer(m,height=36,width=64);renderer.enable_depth_rendering()
    option=mujoco.MjvOption();option.geomgroup[:]=0;option.geomgroup[[1,2]]=1
    def state():
        return RobotState(d.qpos[maps.qposadr].copy(),d.qvel[maps.qveladr].copy(),d.qpos[3:7].copy(),
            d.qvel[3:6].copy(),np.zeros(23),np.zeros(23,np.uint32),0,np.zeros(4),4,step,
            1_000_000_000+round(d.time*1e9))
    c=Controller(p,cfg['control']);c.phase=Phase.SETTLING;c.phase_start_ns=1_000_000_000
    c.settling_start_ns=1_000_000_000
    c.last_target=p.default_q.copy();c.last_time_ns=1_000_000_000
    target=p.default_q.copy();action=None;max_error=0.;min_z=.8;points=[]
    report={'synthetic':True,'warmup_s':warmup,'policy_blend_s':cfg['control']['policy_blend_s'],
            'target_rate_limit_rad_s':cfg['control']['max_target_step_rad_s'],'executed_history':executed_history}
    try:
        for step in range(1,round(seconds/.005)+1):
            d.ctrl[maps.actuator]=sim.pd_control(target,d.qpos[maps.qposadr],kp,np.zeros(23),d.qvel[maps.qveladr],kd)
            mujoco.mj_step(m,d)
            s=state();now=s.received_at_ns
            validate_state(s,now,cfg['control'],p,cfg['expected_mode_machine'])
            if c.phase==Phase.READY and d.time>=warmup:
                c.begin_policy(s,now);p.reset()
                report['entry_q_error_rad']=float(np.max(np.abs(s.q-p.default_q)))
                report['entry_quaternion_wxyz']=s.quaternion_wxyz.tolist()
                report['entry_q_rad']=s.q.tolist()
                report['entry_dq_rad_s']=s.dq.tolist()
            if c.phase==Phase.POLICY and step%4==0:
                renderer.update_scene(d,camera='depth_cam',scene_option=option)
                if executed_history:
                    previous_policy_q=target[p.env.joint_ids_map]
                    p.env.raw_action[:]=(previous_policy_q-np.asarray(p.metadata['action_offset_rad']))/np.asarray(p.metadata['action_scale'])
                raw,desired=p.step(s,np.zeros(3,np.float32),renderer.render())
                action=ActionSample(desired,raw,now,now,now,c.generation)
            output=c.output(s,now,action)
            target,kp,kd=output
            err=float(np.max(np.abs(target-s.q)));max_error=max(max_error,err);min_z=min(min_z,float(d.qpos[2]))
            if step%100==0:points.append({'t':d.time,'z':float(d.qpos[2]),'max_error_rad':err})
            if d.qpos[2]<.5:
                report.update(status='fallen',time_s=d.time);break
        else:report.update(status='PASS' if c.phase==Phase.POLICY else 'NO_POLICY_ENTRY',time_s=d.time)
    except Exception as exc:
        report.update(status='FAIL',error=str(exc),time_s=d.time)
        if isinstance(exc,JointTrackingError):report['tracking_error']=exc.details
    finally:renderer.close()
    report.update(max_error_rad=max_error,min_z=min_z,points=points)
    print(json.dumps({k:v for k,v in report.items() if k not in ('points','tracking_error')}),flush=True)
    return report

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--warmup',type=float,default=.5)
    parser.add_argument('--blend',type=float,default=1.)
    parser.add_argument('--seconds',type=float,default=10.)
    parser.add_argument('--executed-history',action='store_true',help='Diagnostic ablation only')
    parser.add_argument('--report',type=Path,default=root/'logs/sim2real/grounded_handover_validation.json')
    args=parser.parse_args()
    if (not np.isfinite([args.warmup,args.blend,args.seconds]).all() or
            args.blend<=0 or args.warmup<0 or args.seconds<=args.warmup):
        parser.error('Require finite blend>0 and seconds>warmup>=0')
    result=run(args.warmup,args.blend,args.executed_history,args.seconds)
    args.report.parent.mkdir(parents=True,exist_ok=True)
    args.report.write_text(json.dumps(result,indent=2)+'\n')
    raise SystemExit(0 if result['status']=='PASS' else 1)
