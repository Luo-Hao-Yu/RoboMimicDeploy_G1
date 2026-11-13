import copy
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import yaml

from common.path_controller import PathController, PathSettings, PoseSample, PoseUnavailableError
from deploy_real.path_control import RealPathControl
from deploy_real.real_config import load_config


def imu_state(yaw, stamp=1.):
    return SimpleNamespace(quaternion_wxyz=np.array([math.cos(yaw/2), 0, 0, math.sin(yaw/2)]),
                           received_at_ns=round(stamp*1e9))


class RealHeadingTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config()
        self.settings = dict(self.cfg['path_control'], enabled=True, mode='heading', reference='current')

    def adapter(self):
        return RealPathControl(self.settings, self.cfg['control'])

    def test_default_uses_imu_without_opening_localization_file(self):
        self.assertTrue(self.cfg['path_control']['enabled'])
        self.assertEqual(self.cfg['path_control']['mode'], 'heading')
        self.assertNotIn('pose_source', self.cfg['path_control'])
        with patch('deploy_real.path_control.JsonPoseSource', side_effect=AssertionError('External pose requested')):
            adapter = self.adapter()
            np.testing.assert_allclose(adapter.update(imu_state(.7), [.3, 0, 0], 1.), [.3, 0, 0])
            actual = adapter.update(imu_state(.8, 1.02), [.3, 0, 0], 1.02)
        self.assertAlmostEqual(actual[2], -.2, places=6)
        diagnostic = adapter.controller.diagnostics
        self.assertEqual(diagnostic['state_source'], 'lowstate_imu')
        self.assertIsNone(diagnostic['cross_track_m'])
        self.assertIsNone(diagnostic['origin_xy'])
        self.assertAlmostEqual(diagnostic['heading_error_rad'], -.1)

    def test_wraparound_and_existing_velocity_limits(self):
        adapter = self.adapter()
        adapter.update(imu_state(math.pi-.02), [.3, 0, 0], 1.)
        command = adapter.update(imu_state(-math.pi+.02, 1.02), [2, 1, 0], 1.02)
        np.testing.assert_allclose(command, [.3, 0, -.08], atol=1e-6)
        command = adapter.update(imu_state(0, 1.04), [.3, 0, 0], 1.04)
        self.assertAlmostEqual(command[2], .5)
        self.assertTrue(adapter.controller.diagnostics['saturated'])

    def test_manual_turn_stop_restart_and_policy_reset(self):
        adapter = self.adapter()
        adapter.update(imu_state(0), [.3, 0, 0], 1.)
        np.testing.assert_allclose(adapter.update(imu_state(2, 2), [.2, 0, .4], 2), [.2, 0, .4])
        np.testing.assert_allclose(adapter.update(imu_state(3.1, 3), [.3, 0, 0], 3), [.3, 0, 0])
        self.assertLess(adapter.update(imu_state(3.2, 4), [.3, 0, 0], 4)[2], 0)
        np.testing.assert_array_equal(adapter.update(None, [0, 0, 0], 5), 0)
        np.testing.assert_allclose(adapter.update(imu_state(-1, 6), [.3, 0, 0], 6), [.3, 0, 0])
        adapter.reset()
        np.testing.assert_allclose(adapter.update(imu_state(.5, 7), [.3, 0, 0], 7), [.3, 0, 0])

    def test_invalid_stale_future_and_regressed_imu_are_rejected(self):
        for quaternion in (np.zeros(4), np.array([np.nan, 0, 0, 0]), np.ones(3)):
            state = imu_state(0)
            state.quaternion_wxyz = quaternion
            with self.subTest(quaternion=quaternion), self.assertRaises(PoseUnavailableError):
                self.adapter().update(state, [.3, 0, 0], 1.)
        for stamp in (.8, 1.1):
            with self.subTest(stamp=stamp), self.assertRaises(PoseUnavailableError):
                self.adapter().update(imu_state(0, stamp), [.3, 0, 0], 1.)
        adapter = self.adapter()
        adapter.update(imu_state(0), [.3, 0, 0], 1.)
        with self.assertRaises(PoseUnavailableError):
            adapter.update(imu_state(0, .99), [.3, 0, 0], 1.02)

    def test_heading_does_not_reanchor_each_frame_when_xy_is_unknown(self):
        controller = PathController(PathSettings(enabled=True, mode='heading', reference='startup'))
        controller.update(1, PoseSample(1, None, .4), [.3, 0, 0])
        self.assertAlmostEqual(controller.update(2, PoseSample(2, None, .5), [.3, 0, 0])[2], -.2, places=6)
        self.assertAlmostEqual(controller.path_yaw, .4)

    def test_path_mode_still_requires_position(self):
        controller = PathController(PathSettings(enabled=True, mode='path'))
        with self.assertRaisesRegex(PoseUnavailableError, 'requires pelvis XY'):
            controller.update(1, PoseSample(1, None, .4), [.3, 0, 0])
        with self.assertRaisesRegex(ValueError, 'localization source'):
            RealPathControl(dict(self.settings, mode='path'), self.cfg['control'])

    def test_configuration_rejects_path_without_localization_and_excess_limits(self):
        with tempfile.TemporaryDirectory() as directory:
            config_path = Path(directory)/'real.yaml'
            cfg = copy.deepcopy(self.cfg)
            cfg['path_control']['mode'] = 'path'
            config_path.write_text(yaml.safe_dump(cfg))
            with self.assertRaisesRegex(ValueError, 'localization source'):
                load_config(config_path)
            cfg['path_control']['pose_source'] = dict(type='json_file', path='/tmp/example-pose.json')
            config_path.write_text(yaml.safe_dump(cfg))
            self.assertEqual(load_config(config_path)['path_control']['mode'], 'path')
            cfg['path_control']['mode'] = 'heading'
            cfg['path_control']['max_yaw_rad_s'] = 2.
            config_path.write_text(yaml.safe_dump(cfg))
            with self.assertRaisesRegex(ValueError, 'limits'):
                load_config(config_path)

    def test_disabled_feedback_preserves_direct_commands(self):
        adapter = RealPathControl(dict(self.settings, enabled=False), self.cfg['control'])
        np.testing.assert_allclose(adapter.update(None, [.2, 0, -.3], 1.), [.2, 0, -.3])

    def test_inference_worker_writes_corrected_heading_into_policy_history(self):
        from deploy_real.deploy_real import InferenceWorker
        from deploy_real.parkour_policy import ParkourPolicy
        from deploy_real.robot_state import RobotState
        policy = ParkourPolicy('config/g1_23dof/parkour/params/deploy.yaml')
        yaws = [.5, .6, 1.5, 2., 2.1, 2.2]
        sticks = [[0, 1, 0, 0], [0, 1, 0, 0], [0, 1, -.5, 0],
                  [0, 1, 0, 0], [0, 0, 0, 0], [0, 1, 0, 0]]
        class Steps:
            index = 0
            def is_set(inner): return inner.index >= len(yaws)
            def wait(inner, duration): inner.index += 1
        steps = Steps()
        class Transport:
            def snapshot(inner):
                quat = imu_state(yaws[steps.index]).quaternion_wxyz
                return RobotState(policy.default_q.copy(), np.zeros(23, np.float32), quat,
                                  np.zeros(3), np.zeros(23), np.zeros(23, np.uint32), 0,
                                  np.array(sticks[steps.index]), 4, steps.index+1, time.monotonic_ns())
        class Camera:
            def read(inner): return time.monotonic_ns(), np.ones((36, 64), np.float32)
        commands = []
        original_step = policy.step
        def observe(state, command, depth):
            result = original_step(state, command, depth)
            term = next(t for t in policy.env.obs_terms if t.name == 'velocity_commands')
            np.testing.assert_array_equal(term.get_flat().reshape(8, 3)[-1], command)
            commands.append(command.copy())
            return result
        with patch('deploy_real.path_control.JsonPoseSource', side_effect=AssertionError('External pose requested')):
            worker = InferenceWorker(policy, Transport(), Camera(), self.cfg)
            worker.stop = steps
            worker.request(1, True)
            with patch.object(policy, 'step', side_effect=observe):
                worker.run()
        _, count = worker.snapshot()
        self.assertEqual(count, 6)
        self.assertAlmostEqual(commands[0][2], 0)
        self.assertAlmostEqual(commands[1][2], -.2, places=6)
        self.assertGreater(commands[2][2], 0)
        self.assertAlmostEqual(commands[3][2], 0)
        np.testing.assert_array_equal(commands[4], 0)
        self.assertAlmostEqual(commands[5][2], 0)


if __name__ == '__main__':
    unittest.main()
