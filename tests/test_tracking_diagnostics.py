"""Offline tracking-fault diagnostics; no hardware access or motion."""
import json
from pathlib import Path
import tempfile
import threading
import time
from dataclasses import replace
from unittest import TestCase, mock

import numpy as np

from deploy_real.control import Controller, Phase, ActionSample, JointTrackingError
from deploy_real.parkour_policy import ParkourPolicy
from deploy_real.real_config import load_config
from deploy_real.robot_state import RobotState


class TrackingDiagnosticsTests(TestCase):
    @classmethod
    def setUpClass(cls):
        cls.policy = ParkourPolicy('config/g1_23dof/parkour/params/deploy.yaml')
        cls.cfg = load_config()

    def state(self, stamp=1_000_000_000):
        return RobotState(self.policy.default_q.copy(), np.zeros(23), np.array([1.,0,0,0]),
                          np.zeros(3), np.zeros(23), np.zeros(23,dtype=np.uint32),
                          0,np.zeros(4),4,100,stamp)

    def test_fault_names_hardware_slot_and_rejects_candidate(self):
        # left shoulder yaw is compact 15, HG slot 17, policy index 13.
        strict = dict(self.cfg['control'], max_position_error_rad=.8, tracking_recovery=dict(enabled=False))
        c=Controller(self.policy,strict)
        c.phase=Phase.POLICY
        c.generation=1
        c.phase_start_ns=1_000_000_000
        c.last_time_ns=3_000_000_000
        c.last_target=self.policy.default_q.copy()
        c.last_target[15]=.79
        previous=c.last_target.copy()
        target=self.policy.default_q.copy();target[15]=1.2
        raw=np.zeros(23);raw[13]=1.2/self.policy.metadata['action_scale'][13]
        action=ActionSample(target,raw,3_005_000_000,3_005_000_000,3_005_000_000,1)
        with self.assertRaises(JointTrackingError) as caught:
            c.output(self.state(3_005_000_000),3_005_000_000,action)
        report=caught.exception.details
        worst=report['joints'][0]
        self.assertEqual(worst['name'],'left_shoulder_yaw_joint')
        self.assertEqual((worst['compact_index'],worst['motor_slot'],worst['policy_index']),(15,17,13))
        self.assertAlmostEqual(worst['candidate_target_rad'],.82)
        self.assertAlmostEqual(worst['previous_target_rad'],.79)
        self.assertAlmostEqual(worst['error_rad'],.82)
        self.assertFalse(report['candidate_sent'])
        np.testing.assert_array_equal(c.last_target,previous)
        self.assertEqual(c.last_time_ns,3_000_000_000)
        self.assertEqual(len(report['joints']),23)
        self.assertIn('motor 17',str(caught.exception))
        json.dumps(report,allow_nan=False)

    def test_fault_report_saved_after_cleanup_even_without_report_argument(self):
        from deploy_real.deploy_real import run
        state=self.state()
        events=[]
        class Transport:
            def __init__(self,*args,**kwargs):
                assert not kwargs['enable_output']
            def snapshot(self):return replace(state,received_at_ns=time.monotonic_ns())
            def close(self):events.append('transport closed')
        class Camera:
            def read(self):return time.monotonic_ns(),np.ones((36,64),np.float32)
            def close(self):
                events.append('camera close attempted')
                raise RuntimeError('test cleanup failure')
        class Worker:
            def __init__(self,*args):self.thread=mock.Mock()
            def request(self,*args):pass
            def snapshot(self):raise RuntimeError('injected inference failure')
            def statistics(self):return {'policy_steps':1}
            def close(self):events.append('worker closed')
        with tempfile.TemporaryDirectory() as directory, \
             mock.patch('deploy_real.deploy_real.ROOT',Path(directory)), \
             mock.patch('deploy_real.unitree_transport.UnitreeTransport',Transport), \
             mock.patch('deploy_real.deploy_real.create_depth_source',return_value=Camera()), \
             mock.patch('deploy_real.deploy_real.InferenceWorker',Worker):
            with self.assertRaisesRegex(RuntimeError,'injected inference failure'):
                run(self.cfg,self.policy,False,.1)
            files=list(Path(directory).glob('logs/sim2real/real_failure_*.json'))
            self.assertEqual(len(files),1)
            result=json.loads(files[0].read_text())
            self.assertEqual(result['status'],'FAIL')
            self.assertEqual(result['error'],'injected inference failure')
            self.assertEqual(result['phase_at_failure'],'waiting')
            self.assertIsInstance(result['failure_monotonic_ns'],int)
            self.assertNotIn('shutdown',result)
            self.assertEqual(result['cleanup_errors'],['test cleanup failure'])
        self.assertEqual(events,['worker closed','camera close attempted','transport closed'])

    def test_saved_inputs_replay_the_exact_fault_action_without_hardware(self):
        from scripts.replay_real_failure import inspect_report
        p=self.policy
        p.reset()
        state=self.state(3_000_000_000)
        command=np.zeros(3,np.float32)
        raw,target=p.step(state,command,np.ones((36,64),np.float32))
        action=ActionSample(target.copy(),raw.copy(),3_000_000_000,3_000_000_000,3_000_000_000,1,
                            command=command,proprio=p.last_proprio,depth_history=p.last_depth)
        strict = dict(self.cfg['control'], max_position_error_rad=.8, tracking_recovery=dict(enabled=False))
        c=Controller(p,strict);c.phase=Phase.POLICY;c.generation=1
        c.phase_start_ns=1_000_000_000;c.last_time_ns=2_995_000_000
        c.last_target=np.clip(target,p.position_limits[:,0],p.position_limits[:,1])
        q=c.last_target.copy();q[15]-=1.0
        with self.assertRaises(JointTrackingError) as caught:
            c.output(replace(state,q=q),3_000_000_000,action)
        report=dict(error=str(caught.exception),tracking_error=caught.exception.details,config=self.cfg,
                    last_inference=dict(command=command.tolist(),proprio=action.proprio.tolist(),
                                        depth_history=action.depth_history.tolist()))
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'fault.json';path.write_text(json.dumps(report))
            inspect_report(path)

    def test_fault_shutdown_records_feedback_after_rejecting_policy_output(self):
        from deploy_real.deploy_real import run
        from deploy_real.control import START
        cfg=load_config();cfg['control']['damping_duration_s']=.025
        template=self.state()
        created=[]
        class Transport:
            def __init__(inner,*args,**kwargs):
                inner.lock=threading.Lock();inner.latest=None;inner.calls=0
                inner.damping=0;inner.released=False;created.append(inner)
            def snapshot(inner):
                inner.calls+=1
                inner.latest=replace(template,received_at_ns=time.monotonic_ns(),
                                     buttons=START if inner.calls>=3 else 0)
                return inner.latest
            def release_control(inner):inner.released=True
            def send(inner,*args,**kwargs):raise AssertionError('Rejected output was sent')
            def send_damping(inner,kd,mode_machine):
                self.assertEqual(kd,cfg['control']['damping_kd'])
                self.assertEqual(mode_machine,4)
                inner.damping+=1
            def close(inner):pass
        class Camera:
            def read(inner):return time.monotonic_ns(),np.ones((36,64),np.float32)
            def close(inner):pass
        class Worker:
            def __init__(inner,*args):inner.thread=mock.Mock()
            def request(inner,*args):pass
            def snapshot(inner):
                now=time.monotonic_ns()
                return ActionSample(template.q.copy(),np.zeros(23),now,now,now,0),1
            def statistics(inner):return {'policy_steps':1}
            def close(inner):pass
        def fail_after_takeover(controller,*args):
            if controller.phase==Phase.WAITING:return None
            raise ValueError('injected tracking fault')
        with tempfile.TemporaryDirectory() as directory, \
             mock.patch('deploy_real.deploy_real.ROOT',Path(directory)), \
             mock.patch('deploy_real.unitree_transport.UnitreeTransport',Transport), \
             mock.patch('deploy_real.deploy_real.create_depth_source',return_value=Camera()), \
             mock.patch('deploy_real.deploy_real.InferenceWorker',Worker), \
             mock.patch.object(Controller,'output',autospec=True,side_effect=fail_after_takeover):
            with self.assertRaisesRegex(ValueError,'injected tracking fault'):
                run(cfg,self.policy,True,.1)
            report=json.loads(next(Path(directory).glob('logs/sim2real/real_failure_*.json')).read_text())
        self.assertTrue(created[0].released)
        shutdown=report['shutdown']
        self.assertGreater(shutdown['successful_sends'],0)
        self.assertEqual(shutdown['successful_sends'],created[0].damping)
        self.assertEqual(shutdown['send_attempts'],created[0].damping)
        self.assertGreaterEqual(shutdown['start_monotonic_ns'],report['failure_monotonic_ns'])
        self.assertGreater(shutdown['end_monotonic_ns'],shutdown['start_monotonic_ns'])
        self.assertEqual(shutdown['feedback'][0]['quaternion_wxyz'],template.quaternion_wxyz.tolist())
        self.assertEqual(shutdown['stop_reason'],'interval_complete')
