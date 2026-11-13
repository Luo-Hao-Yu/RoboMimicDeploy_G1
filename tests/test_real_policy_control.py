import copy
from dataclasses import replace
import time
import unittest
import threading
import tempfile
from unittest.mock import patch
from pathlib import Path
import numpy as np

from deploy_real.parkour_policy import ParkourPolicy
from deploy_real.robot_state import RobotState
from deploy_real.real_config import load_config
from deploy_real.control import Controller, Phase, ActionSample, validate_state, velocity_command, START, A
from deploy_real.unitree_transport import fill_motor_command
from deploy_mujoco.deploy_mujoco_env import DeployEnv, _deploy_env_cfg_for_interface, DepthHistoryBuffer, _run_depth_encoder_actor_onnx
from deploy_real.depth_source import preprocess_depth


class RealPolicyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.policy=ParkourPolicy('config/g1_23dof/parkour/params/deploy.yaml')
        cls.cfg=load_config()['control']

    def state(self, now=1_000_000_000):
        p=self.policy
        return RobotState(p.default_q.copy(),np.zeros(23,np.float32),np.array([1,0,0,0],np.float32),
                          np.zeros(3,np.float32),np.zeros(23),np.zeros(23,np.uint32),0,np.zeros(4),4,1,now)

    def test_real_observations_actions_match_simulation_for_64_frames(self):
        p=self.policy;p.reset()
        sim=DeployEnv(_deploy_env_cfg_for_interface(p.cfg,'depth_encoder_actor'))
        sim.reset();depth_hist=DepthHistoryBuffer(37,(18,32))
        rng=np.random.default_rng(23)
        for i in range(64):
            q=p.default_q+rng.normal(0,.02,23).astype(np.float32)
            dq=rng.normal(0,.05,23).astype(np.float32)
            quaternion=rng.normal(0,.1,4).astype(np.float32);quaternion[0]+=1;quaternion/=np.linalg.norm(quaternion)
            state=replace(self.state(),q=q,dq=dq,quaternion_wxyz=quaternion,gyro=rng.normal(0,.1,3).astype(np.float32))
            command=np.array([.2,0,-.1],np.float32)
            depth=rng.uniform(.05,3,(36,64)).astype(np.float32)
            # Pack compact motor data into simulated qpos/qvel, using the same PR joints.
            qpos=np.r_[np.zeros(3),quaternion,q];qvel=np.r_[np.zeros(3),state.gyro,dq]
            sim.sync_robot_from_mujoco(qpos,qvel,np.arange(7,30),np.arange(6,29))
            sim.robot.velocity_command[:]=command
            depth_hist.append(preprocess_depth(depth,p.depth_cfg))
            obs=sim.compute_obs()['obs'];d=depth_hist.delayed_frames(8,5,0)
            expected_action=_run_depth_encoder_actor_onnx(p.encoder,p.actor,obs,d)
            sim.process_action(expected_action)
            actual,target=p.step(state,command,depth)
            np.testing.assert_allclose(p.last_proprio,obs,atol=2e-6)
            np.testing.assert_array_equal(p.last_depth,d)
            np.testing.assert_allclose(actual,expected_action,atol=1e-5)
            np.testing.assert_allclose(target,sim.target_sdk_joint_pos(),atol=1e-5)
            if i==31:p.reset();sim.reset();depth_hist=DepthHistoryBuffer(37,(18,32))

    def test_sparse_slots_and_serialized_dq(self):
        import os
        os.environ.setdefault('CYCLONEDDS_HOME',str(Path(__file__).resolve().parents[1]/'.deps/cyclonedds'))
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        msg=unitree_hg_msg_dds__LowCmd_()
        q=np.arange(23)*.01
        fill_motor_command(msg,self.policy.motor_indices,q,self.policy.kp,self.policy.kd,4)
        restored=type(msg).deserialize(msg.serialize())
        for slot in range(35):
            motor=restored.motor_cmd[slot]
            if slot in self.policy.motor_indices:
                i=self.policy.motor_indices.index(slot)
                self.assertEqual(motor.mode,1);self.assertAlmostEqual(motor.q,q[i],places=6)
                self.assertEqual(motor.dq,0);self.assertEqual(motor.tau,0)
            else:self.assertEqual((motor.mode,motor.kp,motor.kd,motor.tau),(0,0,0,0))
        self.assertEqual((restored.mode_pr,restored.mode_machine),(0,4))

    def test_state_watchdogs_and_validation(self):
        s=self.state();p=self.policy;c=self.cfg
        validate_state(s,s.received_at_ns,c,p,4)
        bad=[(replace(s,mode_machine=1),s.received_at_ns),
             (s,s.received_at_ns+200_000_000),
             (replace(s,gyro=np.array([np.nan,0,0])),s.received_at_ns),
             (replace(s,quaternion_wxyz=np.zeros(4)),s.received_at_ns),
             (replace(s,motor_errors=np.ones(23,np.uint32)),s.received_at_ns),
             (replace(s,temperature_c=np.full(23,100)),s.received_at_ns)]
        for state,now in bad:
            with self.subTest(state=state),self.assertRaises(ValueError):validate_state(state,now,c,p,4)

    def test_ramp_ready_policy_blend_stale_action_and_fault(self):
        s=self.state();c=Controller(self.policy,self.cfg);now=s.received_at_ns
        self.assertEqual(c.buttons(replace(s,buttons=START)),0)
        c.buttons(s);self.assertEqual(c.buttons(replace(s,buttons=START)),START)
        c.begin_ramp(s,now)
        for i in range(1,705):
            t=now+i*5_000_000
            target,kp,kd=c.output(replace(s,received_at_ns=t),t)
        self.assertEqual(c.phase,Phase.READY)
        c.begin_policy(replace(s,received_at_ns=t),t)
        self.assertEqual(c.generation,1)
        c.output(s,t+5_000_000,None)
        with self.assertRaisesRegex(ValueError,'timed out'):c.output(s,t+110_000_000,None)
        sample=ActionSample(self.policy.default_q+.1,np.ones(23),t+10_000_000,t+10_000_000,t+10_000_000,1)
        target,kp,kd=c.output(s,t+10_000_000,sample)
        self.assertLess(np.max(np.abs(target-self.policy.default_q)),.01)
        with self.assertRaisesRegex(ValueError,'Stale'):c.output(s,t+300_000_000,sample)
        c.fault('timeout');out=c.output(s,t+310_000_000,sample)
        self.assertTrue((out[1]==0).all());self.assertTrue((out[2]==self.cfg['damping_kd']).all())
        with self.assertRaises(ValueError):c.begin_policy(s,t+320_000_000)

    def test_monitor_loop_with_real_onnx_never_publishes(self):
        from deploy_real.deploy_real import run
        state_template=self.state()
        class Transport:
            def __init__(self,*args,**kwargs):
                self.lock=threading.Lock();self.latest=None;self.closed=False
                self.assert_monitor=not kwargs['enable_output']
            def snapshot(self):
                self.latest=replace(state_template,received_at_ns=time.monotonic_ns())
                return self.latest
            def send(self,*args,**kwargs):raise AssertionError('Monitor attempted motor output')
            def release_control(self,*args,**kwargs):raise AssertionError('Monitor attempted takeover')
            def close(self):self.closed=True
        class Camera:
            def read(self):return time.monotonic_ns(),np.ones((36,64),np.float32)
            def close(self):pass
        cfg=load_config()
        with patch('deploy_real.unitree_transport.UnitreeTransport',Transport), \
             patch('deploy_real.deploy_real.create_depth_source',return_value=Camera()):
            run(cfg,self.policy,False,.15)

    def test_enabled_loop_requires_start_and_ready_a_before_policy(self):
        from deploy_real.deploy_real import run
        cfg=load_config()
        cfg['control'].update(ramp_s=.03,policy_blend_s=.03,damping_duration_s=.01,ready_hold_s=.02)
        state_template=self.state()
        created=[]
        class Transport:
            def __init__(self,*args,**kwargs):
                self.lock=threading.Lock();self.latest=None;self.created=time.monotonic()
                self.released=False;self.commands=[];self.damping=0
                created.append(self)
            def snapshot(self):
                elapsed=time.monotonic()-self.created
                buttons=START if .04<elapsed<.07 else (A if .12<elapsed<.15 else 0)
                self.latest=replace(state_template,buttons=buttons,received_at_ns=time.monotonic_ns())
                return self.latest
            def release_control(self):self.released=True
            def send(self,*args,**kwargs):
                if not self.released:raise AssertionError('Output before explicit handover')
                self.commands.append(args)
            def send_damping(self,*args):self.damping+=1
            def close(self):pass
        class Camera:
            def read(self):return time.monotonic_ns(),np.ones((36,64),np.float32)
            def close(self):pass
        with tempfile.TemporaryDirectory() as directory, \
             patch('deploy_real.deploy_real.ROOT',Path(directory)), \
             patch('deploy_real.unitree_transport.UnitreeTransport',Transport), \
             patch('deploy_real.deploy_real.create_depth_source',return_value=Camera()):
            run(cfg,self.policy,True,.25)
        transport=created[0]
        self.assertTrue(transport.released)
        self.assertGreater(len(transport.commands),10)
        self.assertGreater(transport.damping,0)
        self.assertTrue(any(np.max(np.abs(cmd[0]-self.policy.default_q))>1e-4 for cmd in transport.commands))

    def test_direct_remote_command_without_heading_feedback(self):
        s=replace(self.state(),sticks=np.array([.8,1,.5,0]))
        command=velocity_command(s,self.cfg)
        self.assertAlmostEqual(command[0],.3,places=6);self.assertEqual(command[1],0)
        self.assertLess(command[2],0)
        np.testing.assert_array_equal(velocity_command(replace(s,sticks=np.zeros(4)),self.cfg),0)


if __name__=='__main__':unittest.main()
