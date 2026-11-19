import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from dataclasses import replace

import numpy as np

from common.path_controller import (PathController, PathSettings, PathState,
                                    PoseSample, PoseUnavailableError, yaw_wxyz)
from deploy_real.path_control import JsonPoseSource, RealPathControl


class PathControlTests(unittest.TestCase):
    def setUp(self):
        self.cfg = PathSettings(enabled=True)
        self.pose = PoseSample(1., (0., 0.), 0.)

    def test_signed_cross_track_rotated_frame_and_angle_wrap(self):
        c = PathController(self.cfg)
        command = c.update(1., replace(self.pose, xy=(0., .2)), [.4, 0, 0])
        self.assertAlmostEqual(command[2], -2*math.atan2(.2, 1), places=6)
        rotated = PathController(replace(self.cfg, origin_xy=(2., 3.), yaw_rad=math.pi/2))
        command = rotated.update(1., replace(self.pose, xy=(1.8, 3.), yaw=math.pi/2), [.4, 0, 0])
        self.assertAlmostEqual(command[2], -2*math.atan2(.2, 1), places=6)
        c = PathController(replace(self.cfg, yaw_rad=math.pi-.01))
        command = c.update(1., replace(self.pose, yaw=-math.pi+.01), [.4, 0, 0])
        self.assertAlmostEqual(command[2], -.04, places=6)

    def test_stop_and_restart_preserve_fixed_path(self):
        c = PathController(self.cfg)
        c.update(1., self.pose, [.4, 0, 0])
        np.testing.assert_array_equal(c.update(2., None, [0, 0, 0]), 0)
        self.assertEqual(c.state, PathState.STOP)
        command = c.update(3., replace(self.pose, timestamp_s=3., xy=(0., .3)), [.4, 0, 0])
        self.assertLess(command[2], 0)
        np.testing.assert_array_equal(c.origin, [0, 0])

    def test_manual_priority_and_explicit_open_ground_rebase(self):
        c = PathController(self.cfg)
        np.testing.assert_allclose(c.update(1., self.pose, [.4, .5, .7]), [.4, 0, .7])
        self.assertEqual(c.state, PathState.MANUAL_TURN)
        pose = replace(self.pose, timestamp_s=2., xy=(1., .4), yaw=.2)
        self.assertLess(c.update(2., pose, [.4, 0, 0])[2], 0)
        np.testing.assert_array_equal(c.origin, [0, 0])
        open_ground = PathController(replace(self.cfg, reference='startup', manual_rebase=True))
        open_ground.update(1., self.pose, [.4, 0, .7])
        open_ground.update(1.5, None, [0, 0, 0])
        np.testing.assert_allclose(open_ground.update(2., pose, [.4, 0, 0]), [.4, 0, 0])
        np.testing.assert_allclose(open_ground.origin, pose.xy)

    def test_startup_anchor_reset_and_limits(self):
        c = PathController(replace(self.cfg, reference='startup'))
        c.reset(self.pose)
        command = c.update(1., replace(self.pose, xy=(0, 10)), [2, 4, 0])
        np.testing.assert_allclose(command, [.65, 0, -1])
        self.assertTrue(c.diagnostics['saturated'])
        c.reset(replace(self.pose, timestamp_s=0, xy=(0, 10)))
        np.testing.assert_allclose(c.update(0, replace(self.pose, timestamp_s=0, xy=(0, 10)), [.4, 0, 0]), [.4, 0, 0])

    def test_invalid_stale_future_low_confidence_and_frame_changes(self):
        invalid = [None, replace(self.pose, valid=False), replace(self.pose, confidence=.1),
                   replace(self.pose, timestamp_s=.8), replace(self.pose, timestamp_s=1.2),
                   replace(self.pose, xy=(0, float('nan'))), replace(self.pose, frame_id='relocated')]
        for pose in invalid:
            with self.subTest(pose=pose), self.assertRaises(PoseUnavailableError):
                PathController(self.cfg).update(1., pose, [.4, 0, 0])
        c = PathController(self.cfg)
        c.update(1., self.pose, [.4, 0, 0])
        with self.assertRaises(PoseUnavailableError):
            c.update(1.01, replace(self.pose, timestamp_s=.99), [.4, 0, 0])

    def test_disabled_mode_preserves_manual_and_heading_baseline(self):
        c = PathController(PathSettings())
        np.testing.assert_allclose(c.update(1., None, [.3, 0, -.2]), [.3, 0, -.2])
        c = PathController(replace(self.cfg, mode='heading'))
        np.testing.assert_allclose(c.update(1., replace(self.pose, xy=(0, 3)), [.4, 0, 0]), [.4, 0, 0])

    def test_real_json_source_and_state_alignment(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'pose.json'
            data = dict(monotonic_ns=1_000_000_000, xy_m=[0, .2], yaw_rad=0,
                        frame_id='local', valid=True, confidence=.95)
            path.write_text(json.dumps(data))
            cfg = dict(enabled=True, pose_source=dict(type='json_file', path=str(path)))
            control = dict(max_forward_m_s=.65, max_yaw_rad_s=1.)
            c = RealPathControl(cfg, control)
            state = SimpleNamespace(received_at_ns=1_000_000_000)
            self.assertLess(c.update(state, [.4, 0, 0], 1.)[2], 0)
            with self.assertRaises(PoseUnavailableError):
                c.update(SimpleNamespace(received_at_ns=1_060_000_000), [.4, 0, 0], 1.06)
            path.write_text('{')
            with self.assertRaises(PoseUnavailableError):
                JsonPoseSource(path).snapshot()
            np.testing.assert_array_equal(c.update(state, [0, 0, 0], 2.), 0)

    def test_bad_settings(self):
        for cfg in [dict(lookahead_m=0), dict(heading_gain=float('nan')),
                    dict(origin_xy=(0,)), dict(manual_rebase=True), dict(enabled='false')]:
            with self.subTest(cfg=cfg), self.assertRaises(ValueError):
                PathSettings(**cfg)

    def test_pelvis_yaw(self):
        self.assertAlmostEqual(yaw_wxyz([math.cos(.3), 0, 0, math.sin(.3)]), .6)

    def test_current_heading_is_captured_at_start_not_reset_or_every_frame(self):
        c = PathController(replace(self.cfg, reference='current'))
        c.reset(self.pose)
        start = replace(self.pose, timestamp_s=2., xy=(4., 2.), yaw=math.pi/2)
        np.testing.assert_allclose(c.update(2., start, [.4, 0, 0]), [.4, 0, 0])
        # Left of a northbound path: correct clockwise, retaining the old line.
        drifted = replace(start, timestamp_s=3., xy=(3.8, 3.), yaw=math.pi/2+.1)
        command = c.update(3., drifted, [.4, 0, 0])
        self.assertAlmostEqual(command[2], 2*(-math.atan2(.2, 1)-.1), places=6)
        np.testing.assert_allclose(c.origin, start.xy)
        self.assertAlmostEqual(c.path_yaw, start.yaw)

    def test_current_heading_turnaround_and_return_correct_in_new_frame(self):
        for turn_sign in (-1, 1):
            with self.subTest(turn_sign=turn_sign):
                c = PathController(replace(self.cfg, reference='current'))
                c.update(1., self.pose, [.4, 0, 0])
                turn = replace(self.pose, timestamp_s=2., xy=(8., .2), yaw=turn_sign*2.)
                np.testing.assert_allclose(c.update(2., turn, [0, 0, turn_sign*.5]), [0, 0, turn_sign*.5])
                back = replace(turn, timestamp_s=3., yaw=turn_sign*(math.pi-.01))
                np.testing.assert_allclose(c.update(3., back, [.4, 0, 0]), [.4, 0, 0])
                normal = np.array([-math.sin(back.yaw), math.cos(back.yaw)])
                direction = np.array([math.cos(back.yaw), math.sin(back.yaw)])
                returned = replace(back, timestamp_s=4., xy=tuple(np.array(back.xy)+direction+.2*normal))
                self.assertLess(returned.xy[0], back.xy[0])
                self.assertAlmostEqual(c.update(4., returned, [.4, 0, 0])[2], -2*math.atan2(.2, 1), places=6)
                self.assertAlmostEqual(c.path_yaw, back.yaw)

    def test_current_heading_stop_resume_and_turn_release_while_walking(self):
        c = PathController(replace(self.cfg, reference='current'))
        c.update(1., self.pose, [.4, 0, 0])
        turning = replace(self.pose, timestamp_s=2., yaw=.7)
        np.testing.assert_allclose(c.update(2., turning, [.4, 0, .2]), [.4, 0, .2])
        released = replace(turning, timestamp_s=3., yaw=1.2)
        np.testing.assert_allclose(c.update(3., released, [.4, 0, 0]), [.4, 0, 0])
        np.testing.assert_array_equal(c.update(4., None, [0, 0, 0]), 0)
        resumed = replace(released, timestamp_s=5., xy=(2., 3.), yaw=-2.)
        np.testing.assert_allclose(c.update(5., resumed, [.4, 0, 0]), [.4, 0, 0])
        np.testing.assert_allclose(c.origin, resumed.xy)
        self.assertEqual(c.path_yaw, -2.)

    def test_right_stick_deadzone_keeps_current_segment_reference(self):
        from deploy_mujoco.gamepad_control import axis_velocity
        c = PathController(replace(self.cfg, reference='current'))
        c.update(1., self.pose, [.4, 0, 0])
        for index, stick in enumerate([.01, -.05, .1, -.11], start=2):
            pose = replace(self.pose, timestamp_s=float(index), xy=(index*.1, .2))
            wz = axis_velocity(stick, [-1, 1], .12)
            self.assertEqual(wz, 0)
            self.assertLess(c.update(float(index), pose, [.4, 0, wz])[2], 0)
            np.testing.assert_array_equal(c.origin, [0, 0])

    def test_current_heading_sim_real_adapters_agree_after_turn(self):
        import mujoco
        from deploy_mujoco.path_control import SimulationPathControl
        root = Path(__file__).resolve().parents[1]
        model = mujoco.MjModel.from_xml_path(str(root/'g1_description/g1_23dof_parkour/scene.xml'))
        data = mujoco.MjData(model)
        cfg = dict(enabled=True, reference='current')
        sim = SimulationPathControl(cfg, model, data)
        class Source:
            def snapshot(self): return sim.pose()
        real = RealPathControl(cfg, dict(max_forward_m_s=.65, max_yaw_rad_s=1.), Source())
        for stamp, x, y, yaw, intent in [
            (1, 0, 0, .5, [.4, 0, 0]), (2, 2, .2, 2., [.4, 0, .5]),
            (3, 2, .2, math.pi, [.4, 0, 0]), (4, 1, 0, math.pi, [.4, 0, 0]),
            (5, 1, 0, math.pi, [0, 0, 0]), (6, 1, 0, -1., [.4, 0, 0])]:
            data.time = stamp
            data.qpos[:2] = [x, y]
            data.qpos[3:7] = [math.cos(yaw/2), 0, 0, math.sin(yaw/2)]
            state = SimpleNamespace(received_at_ns=stamp*1_000_000_000)
            a, b = sim.update(intent), real.update(state, intent, stamp)
            np.testing.assert_array_equal(a, b)
            if stamp in (1, 3, 6):
                self.assertAlmostEqual(a[2], 0)
            if stamp == 4:
                self.assertLess(a[2], 0)


if __name__ == '__main__':
    unittest.main()
