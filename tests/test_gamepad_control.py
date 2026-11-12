import struct
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import yaml

from deploy_mujoco.gamepad_control import GamepadControl, axis_velocity

RANGES = {'lin_vel_x': [-0.5, 1.0], 'lin_vel_y': [-0.3, 0.3], 'ang_vel_z': [-1.0, 1.0]}


def axis(number, value, initial=False):
    return struct.pack('=IhBB', 0, value, 0x82 if initial else 0x02, number)


class GamepadTests(unittest.TestCase):
    def setUp(self):
        self.pad = GamepadControl(RANGES)
        self.pad._fd = 7
        self.pad._axis_indices = [1, 0, 3]
        self.command = np.ones(3, dtype=np.float32)

    def poll(self, *packets):
        with patch('deploy_mujoco.gamepad_control.os.read', side_effect=[*packets, BlockingIOError()]):
            self.pad.update(self.command)
        return self.command.copy()

    def center(self):
        self.poll(axis(0, 0, True), axis(1, 0, True), axis(3, 0, True))

    def test_deadzone_and_asymmetric_limits(self):
        for value in [-0.12, 0, 0.12]:
            self.assertEqual(axis_velocity(value, [-0.5, 1], 0.12), 0)
        self.assertAlmostEqual(axis_velocity(0.56, [-0.5, 1], 0.12), 0.5)
        self.assertEqual(axis_velocity(-1, [-0.5, 1], 0.12), -0.5)
        self.assertEqual(axis_velocity(1, [0, 0], 0.12), 0)

    def test_left_forward_left_and_right_turn(self):
        self.center()
        cmd = self.poll(axis(1, -32768), axis(0, -32768), axis(3, 32767))
        np.testing.assert_allclose(cmd, [1, 0.3, -1])
        cmd = self.poll(axis(1, 32767), axis(0, 32767), axis(3, -32768))
        np.testing.assert_allclose(cmd, [-0.5, -0.3, 1])

    def test_release_and_ignore_other_axes_and_buttons(self):
        self.center()
        self.poll(axis(1, -32768))
        cmd = self.poll(axis(1, 0), axis(2, 32767), axis(4, -32768),
                        struct.pack('=IhBB', 0, 1, 0x01, 0))
        np.testing.assert_array_equal(cmd, 0)

    def test_waits_for_initial_events_and_neutral(self):
        np.testing.assert_array_equal(self.poll(axis(1, -32768, True)), 0)
        np.testing.assert_array_equal(self.poll(axis(0, 0, True), axis(3, 0, True)), 0)
        self.assertFalse(self.pad._armed)
        self.center()
        self.assertTrue(self.pad._armed)
        np.testing.assert_allclose(self.poll(axis(1, -32768)), [1, 0, 0])
        # Held sticks keep their value even if there are no new events.
        np.testing.assert_allclose(self.poll(), [1, 0, 0])

    def test_disconnect_resets_and_reconnect_requires_neutral(self):
        self.center()
        self.poll(axis(1, -32768))
        with patch('deploy_mujoco.gamepad_control.os.read', side_effect=OSError('unplugged')), \
             patch('deploy_mujoco.gamepad_control.os.close') as close:
            self.pad.update(self.command)
            close.assert_called_once_with(7)
        np.testing.assert_array_equal(self.command, 0)
        self.assertFalse(self.pad._armed)
        self.assertIsNone(self.pad._fd)
        with patch.object(self.pad, '_connect') as connect:
            self.pad.update(self.command)
            connect.assert_not_called()
        self.pad._retry_at = 0
        def reconnect():
            self.pad._fd = 8
        with patch.object(self.pad, '_connect', side_effect=reconnect):
            self.poll(axis(0, 0, True), axis(1, -32768, True), axis(3, 0, True))
        np.testing.assert_array_equal(self.command, 0)
        self.assertFalse(self.pad._armed)
        self.center()
        np.testing.assert_allclose(self.poll(axis(1, -32768)), [1, 0, 0])

    def test_missing_device_holds_zero(self):
        self.pad._fd = None
        with patch.object(self.pad, '_connect', side_effect=FileNotFoundError('missing')):
            self.pad.update(self.command)
        np.testing.assert_array_equal(self.command, 0)

    def test_invalid_configuration(self):
        for deadzone in [-0.1, 1, float('nan')]:
            with self.assertRaises(ValueError):
                GamepadControl(RANGES, deadzone=deadzone)
        with self.assertRaises(ValueError):
            GamepadControl(dict(RANGES, lin_vel_x=[0.1, 1]))

    def test_parkour_commands_reach_policy_observation(self):
        from deploy_mujoco.deploy_mujoco_env import _velocity_command_obs
        from deploy_mujoco.deploy_mujoco import build_parser
        root = Path(__file__).resolve().parents[1]
        cfg = yaml.safe_load((root / 'config/g1_23dof/parkour/params/deploy.yaml').read_text())
        self.assertEqual(cfg['control']['mode'], 'gamepad')
        self.assertTrue((root / cfg['xml_path']).is_file())
        self.center()
        self.poll(axis(1, 32767), axis(0, -32768), axis(3, 32767))
        env = SimpleNamespace(cfg=cfg, robot=SimpleNamespace(velocity_command=self.command))
        np.testing.assert_allclose(_velocity_command_obs(env), [0, 0, -1])
        self.assertEqual(build_parser().parse_args(['--control', 'keyboard']).control, 'keyboard')


if __name__ == '__main__':
    unittest.main()
