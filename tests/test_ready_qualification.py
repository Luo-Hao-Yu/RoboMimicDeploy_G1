"""Ready is measured posture qualification, not a countdown or balance guarantee."""
from dataclasses import replace
import unittest
import numpy as np
from deploy_real.control import Controller, Phase, PostureNotReadyError
from deploy_real.parkour_policy import ParkourPolicy
from deploy_real.real_config import load_config
from deploy_real.robot_state import RobotState


class ReadyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p=ParkourPolicy('config/g1_23dof/parkour/params/deploy.yaml')

    def setUp(self):
        self.cfg=load_config()['control']
        self.c=Controller(self.p,self.cfg)
        self.base=1_000_000_000
        self.s=RobotState(self.p.default_q.copy(),np.zeros(23),np.array([1.,0,0,0]),
                          np.zeros(3),np.zeros(23),np.zeros(23,np.uint32),0,np.zeros(4),4,1,self.base)
        self.c.begin_ramp(self.s,self.base)

    def step(self,t,**changes):
        now=self.base+round(t*1e9)
        s=replace(self.s,received_at_ns=now,**changes)
        return self.c.output(s,now)

    def finish_ramp(self):
        self.step(3.)
        self.assertEqual(self.c.phase,Phase.SETTLING)

    def test_timer_alone_cannot_make_ready(self):
        self.finish_ramp()
        q=self.s.q.copy();q[4]-=.2618;q[10]-=.2601
        for t in np.arange(3.005,5.99,.005):self.step(float(t),q=q)
        self.assertEqual(self.c.phase,Phase.SETTLING)
        with self.assertRaises(PostureNotReadyError) as caught:self.step(6.01,q=q)
        self.assertEqual(caught.exception.details['worst_motor_slot'],4)
        with self.assertRaisesRegex(ValueError,'requires Ready'):
            self.c.begin_policy(replace(self.s,q=q),self.base+6_010_000_000)

    def test_fresh_stable_posture_is_required_for_hold_window(self):
        self.finish_ramp()
        for t in np.arange(3.005,3.5,.005):self.step(float(t))
        self.assertEqual(self.c.phase,Phase.SETTLING)
        self.step(3.51)
        self.assertEqual(self.c.phase,Phase.READY)

    def test_motion_resets_window_and_bad_posture_revokes_ready(self):
        self.finish_ramp()
        for t in np.arange(3.005,3.4,.005):self.step(float(t))
        self.step(3.4,gyro=np.array([.3,0,0]))
        for t in np.arange(3.405,3.9,.005):self.step(float(t))
        self.assertEqual(self.c.phase,Phase.SETTLING)
        self.step(3.91);self.assertEqual(self.c.phase,Phase.READY)
        self.step(3.915,dq=np.full(23,.3))
        self.assertEqual(self.c.phase,Phase.SETTLING)

    def test_stale_or_repeated_sample_does_not_qualify(self):
        self.finish_ramp()
        self.step(3.005)
        fixed=replace(self.s,received_at_ns=self.base+3_005_000_000)
        for t in np.arange(3.01,3.8,.005):
            self.c.output(fixed,self.base+round(float(t)*1e9))
        self.assertEqual(self.c.phase,Phase.SETTLING)
        self.assertIsNone(self.c.ready_since_ns)

    def test_a_rechecks_measured_posture(self):
        self.finish_ramp()
        for t in np.arange(3.005,3.52,.005):self.step(float(t))
        self.assertEqual(self.c.phase,Phase.READY)
        with self.assertRaises(PostureNotReadyError):
            self.c.begin_policy(replace(self.s,gyro=np.array([0,.4,0])),self.base+3_520_000_000)
        self.assertEqual(self.c.generation,0)

    def test_stationary_default_lean_is_not_forced_to_zero_pitch(self):
        self.finish_ramp()
        pitch=.12
        quaternion=np.array([np.cos(pitch/2),0,np.sin(pitch/2),0])
        for t in np.arange(3.005,3.52,.005):self.step(float(t),quaternion_wxyz=quaternion)
        self.assertEqual(self.c.phase,Phase.READY)


if __name__=='__main__':unittest.main()
