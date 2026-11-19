"""Offline regression tests for tracking recovery limits and transitions."""
from dataclasses import replace
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np
import yaml

from deploy_real.control import Controller, Phase, ActionSample, JointTrackingError
from deploy_real.real_config import load_config
from deploy_real.robot_state import RobotState


class TrackingRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config()['control']
        self.cfg['max_position_error_rad'] = .8
        self.cfg['tracking_recovery'] = dict(enabled=True, hard_error_rad=1.,
            release_error_rad=.65, timeout_s=.5, release_hold_s=.1)
        self.policy = SimpleNamespace(kp=np.full(23, 28.5), kd=np.full(23, 1.8),
            default_q=np.zeros(23), position_limits=np.tile([-2., 2.], (23,1)),
            env=SimpleNamespace(joint_ids_map=np.arange(23), sdk_joint_names=[f'joint_{i}' for i in range(23)]),
            motor_indices=list(range(23)))
        self.now = 3_000_000_000
        self.controller = Controller(self.policy, self.cfg)
        c = self.controller
        c.phase = Phase.POLICY
        c.generation = 1
        c.phase_start_ns = 1_000_000_000
        c.last_time_ns = self.now
        c.last_target = np.zeros(23)
        self.state = RobotState(np.zeros(23), np.zeros(23), np.array([1.,0,0,0]), np.zeros(3),
                                np.zeros(23), np.zeros(23,np.uint32), 0, np.zeros(4), 4, 1, self.now)

    def step(self, desired=.95, q=0., *, stamp=None, action_age_ns=0):
        self.now += 5_000_000
        state = replace(self.state, q=self.state.q.copy(), received_at_ns=self.now if stamp is None else stamp)
        state.q[4] = q
        target = np.zeros(23); target[4] = desired
        action = ActionSample(target, target.copy(), self.now-action_age_ns,
                              self.now-action_age_ns, self.now, 1)
        previous = self.controller.last_target.copy()
        output = self.controller.output(state, self.now, action)
        return output, previous, state, action

    def trigger(self):
        self.controller.last_target[4] = .79
        return self.step()

    def test_short_exceedance_clips_and_preserves_policy_and_other_joints(self):
        (target,kp,kd), previous, state, action = self.trigger()
        self.assertEqual(self.controller.phase, Phase.POLICY)
        self.assertAlmostEqual(target[4], .8)
        self.assertLessEqual(np.max(np.abs(target-previous)), .03+1e-12)
        np.testing.assert_array_equal(target[np.arange(23)!=4], 0)
        np.testing.assert_array_equal(kp, self.policy.kp)
        np.testing.assert_array_equal(kd, self.policy.kd)
        self.assertEqual(action.target[4], .95)
        self.assertEqual(action.raw_action[4], .95)
        self.assertEqual(self.controller.tracking_guard.statistics()['limited_cycles'], 1)

    def test_negative_error_is_clipped_symmetrically(self):
        self.controller.last_target[4] = -.79
        (target,_,_),_,_,_ = self.step(-.95)
        self.assertAlmostEqual(target[4], -.8)

    def test_persistent_clipping_expires_instead_of_refreshing_timer(self):
        self.trigger()
        started = self.controller.tracking_guard.started_ns
        for _ in range(99):
            self.step()
        previous = self.controller.last_target.copy()
        stamp = self.controller.last_time_ns
        with self.assertRaises(JointTrackingError) as exc:
            self.step()
        self.assertEqual(exc.exception.details['reason'], 'tracking_recovery_timeout')
        self.assertEqual(self.controller.tracking_guard.started_ns, started)
        np.testing.assert_array_equal(self.controller.last_target, previous)
        self.assertEqual(self.controller.last_time_ns, stamp)

    def test_releases_after_fresh_continuous_healthy_demand(self):
        self.trigger()
        for _ in range(35):
            self.step(.5)
        guard = self.controller.tracking_guard
        self.assertFalse(guard.active)
        self.assertEqual(guard.recoveries, 1)
        self.assertEqual(guard.episodes, 1)
        self.assertAlmostEqual(self.controller.last_target[4], .5)
        self.controller.last_target[4] = .79
        self.step()
        self.assertEqual(guard.episodes, 2)

    def test_hysteresis_does_not_release_on_soft_boundary_oscillation(self):
        self.trigger()
        for i in range(50):
            self.step(.79 if i%2 else .82)
        self.assertTrue(self.controller.tracking_guard.active)
        self.assertEqual(self.controller.tracking_guard.episodes, 1)
        self.assertIsNone(self.controller.tracking_guard.healthy_since_ns)

    def test_rate_clipped_demand_cannot_falsely_qualify_recovery(self):
        self.trigger()
        self.controller.last_target[4] = .1
        self.step(.95)
        self.assertIsNone(self.controller.tracking_guard.healthy_since_ns)
        self.assertTrue(self.controller.tracking_guard.active)

    def test_stale_state_and_action_still_fault(self):
        self.trigger()
        with self.assertRaisesRegex(ValueError, 'Stale action'):
            self.step(action_age_ns=200_000_000)
        with self.assertRaises(JointTrackingError) as exc:
            self.step(stamp=self.now-200_000_000)
        self.assertEqual(exc.exception.details['reason'], 'invalid_recovery_state_age')

    def test_repeated_samples_cannot_complete_release_hold(self):
        self.trigger()
        for _ in range(8): self.step(.5)
        fixed_stamp = self.now
        for _ in range(15): self.step(.5, stamp=fixed_stamp)
        self.assertTrue(self.controller.tracking_guard.active)

    def test_large_error_is_immediate_hard_fault(self):
        self.controller.last_target[4] = 1.05
        with self.assertRaises(JointTrackingError) as exc:
            self.step(1.2)
        self.assertEqual(exc.exception.details['reason'], 'hard_tracking_error')
        self.assertEqual(exc.exception.details['limit_rad'], 1.)

    def test_moving_measurement_cannot_force_target_slew_jump(self):
        self.trigger()
        previous = self.controller.last_target.copy()
        with self.assertRaises(JointTrackingError) as exc:
            self.step(.95, q=-.1)
        self.assertEqual(exc.exception.details['reason'], 'infeasible_tracking_rate_position_bounds')
        np.testing.assert_array_equal(self.controller.last_target, previous)

    def test_multiple_joints_are_watched_until_all_recover(self):
        self.trigger()
        self.controller.last_target[5] = -.79
        self.state.q[5] = -1.65
        with self.assertRaises(JointTrackingError) as exc:
            self.step(.5)
        # The new joint cannot satisfy its error and slew constraints together.
        self.assertEqual(exc.exception.details['reason'], 'infeasible_tracking_rate_position_bounds')
        self.assertEqual(self.controller.tracking_guard.diagnostics['watched_indices'], [4,5])

    def test_second_joint_keeps_episode_active_without_restarting_deadline(self):
        self.trigger()
        guard=self.controller.tracking_guard;started=guard.started_ns
        self.controller.last_target[5]=-.79
        self.state.q[5]=-1.6
        self.step(.5)
        self.assertEqual(guard.diagnostics['watched_indices'], [4,5])
        for _ in range(30): self.step(.5)
        self.assertTrue(guard.active)
        self.assertEqual(guard.started_ns,started)
        self.assertEqual(guard.episodes,1)

    def test_measurement_timing_regression_and_position_conflict_fault(self):
        self.trigger()
        with self.assertRaises(JointTrackingError) as exc:
            self.step(stamp=self.now-1_000_000)
        self.assertEqual(exc.exception.details['reason'],'recovery_state_time_regressed')
        self.setUp();self.trigger()
        self.policy.position_limits[4]=[-.1,.1]
        with self.assertRaises(JointTrackingError) as exc: self.step()
        self.assertEqual(exc.exception.details['reason'],'infeasible_tracking_rate_position_bounds')

    def test_new_policy_generation_resets_recovery_state(self):
        self.trigger()
        c=self.controller;c.phase=Phase.READY
        c.begin_policy(replace(self.state,received_at_ns=self.now),self.now)
        self.assertFalse(c.tracking_guard.active)
        self.assertEqual(c.tracking_guard.episodes,0)
        self.assertEqual(c.generation,2)

    def test_strict_mode_and_non_policy_phases_keep_old_behavior(self):
        self.controller = Controller(self.policy, dict(self.cfg, tracking_recovery={'enabled':False}))
        c=self.controller;c.phase=Phase.POLICY;c.generation=1
        c.phase_start_ns=1_000_000_000;c.last_time_ns=self.now;c.last_target=np.zeros(23)
        with self.assertRaises(JointTrackingError) as exc: self.trigger()
        self.assertEqual(exc.exception.details['reason'], 'strict_tracking_error')
        self.setUp()
        c=self.controller;c.phase=Phase.RAMP;c.initial_q=np.zeros(23)
        c.last_target[4]=.9
        with self.assertRaises(JointTrackingError) as exc: self.step()
        self.assertEqual(exc.exception.details['reason'], 'strict_tracking_error')

    def test_invalid_state_or_action_not_hidden_by_recovery(self):
        self.trigger()
        with self.assertRaisesRegex(ValueError, 'Invalid measured'): self.step(q=float('nan'))
        with self.assertRaisesRegex(ValueError, 'Invalid policy output'): self.step(float('nan'))

    def test_config_validation(self):
        for update in [dict(enabled='true'), dict(timeout_s=0), dict(hard_error_rad=.7),
                       dict(release_error_rad=.8), dict(release_hold_s=.6), dict(typo=1),
                       dict(timeout_s=float('nan'))]:
            cfg = load_config()
            cfg['control'] = dict(self.cfg, tracking_recovery=dict(self.cfg['tracking_recovery'], **update))
            with tempfile.TemporaryDirectory() as directory:
                path=Path(directory)/'config.yaml';path.write_text(yaml.safe_dump(cfg))
                with self.subTest(update=update), self.assertRaises(ValueError): load_config(path)

    def test_returned_targets_satisfy_all_three_constraints(self):
        rng=np.random.default_rng(23)
        for _ in range(100):
            self.setUp()
            q=float(rng.uniform(-.8,.8));sign=rng.choice([-1,1])
            self.controller.last_target[4]=q+sign*.79
            (target,_,_),previous,state,_=self.step(q+sign*.95,q)
            self.assertLessEqual(float(np.abs(target-state.q).max()), .8+1e-12)
            self.assertLessEqual(float(np.abs(target-previous).max()), .03+1e-12)
            self.assertTrue(np.all(target>=self.policy.position_limits[:,0]))
            self.assertTrue(np.all(target<=self.policy.position_limits[:,1]))
            json.dumps(self.controller.tracking_guard.statistics(),allow_nan=False)


if __name__ == '__main__': unittest.main()
