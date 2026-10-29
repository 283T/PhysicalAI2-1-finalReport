"""A correction cannot finish at the looser transport tolerance."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from arm_kinematics import axis_rotation
from arm_session import ArmSession


class MotionPrecisionTests(unittest.TestCase):
    def session(self, *, position=.001, angle=.01, improve_at=None):
        session = ArmSession.__new__(ArmSession)
        session.arms = {'right_arm': SimpleNamespace(forward=lambda q: np.eye(4))}
        session.bounds = {'right_arm': Mock()}
        session.fixtures = SimpleNamespace(screen=Mock(return_value={'clear': True}))
        session.preflights = []
        session.targets = {'right_arm': np.zeros(6)}
        session.poses = ({}, 0.)
        session.ticks = 0

        def row():
            improved = improve_at is not None and session.poses[1] >= improve_at
            return {'sim_time_s': session.poses[1], 'arms': {'right_arm': {
                'q_rad': [0.] * 6, 'fingers': [.01, .01],
                'tcp': [.0001 if improved else position, 0, 0],
                'tcp_rotation_world': axis_rotation([0, 0, 1], .001 if improved else angle),
                'fk_native_error_m': 0.}}}

        def tick():
            session.poses = ({}, session.poses[1] + .1)
            session.ticks += 1
            return row()

        session.tick = tick
        session.snapshot = row
        return session

    def move(self, session, precise):
        with patch('arm_session.screen_paths', return_value={'clear': True}):
            return session.move('primary_correction', {'right_arm': np.zeros((2, 6))},
                                require_precision=precise)

    def test_normal_transport_tolerance_remains_unchanged(self):
        session = self.session()
        result = self.move(session, False)
        self.assertLess(result['sim_time_s'], 4)
        self.assertEqual(session.preflights[0]['execution_budget']['outcome'], 'converged')

    def test_position_and_angle_each_prevent_premature_precision_success(self):
        for position, angle in [(.001, .001), (.0001, .01)]:
            with self.subTest(position=position, angle=angle):
                session = self.session(position=position, angle=angle)
                with self.assertRaisesRegex(RuntimeError, 'simulation_budget_exhausted'):
                    self.move(session, True)
                self.assertTrue(session.preflights[0]['require_precision'])
                self.assertEqual(session.preflights[0]['execution_budget']['simulation_budget_s'], 23.)

    def test_waits_for_precision_without_resetting_the_budget(self):
        session = self.session(improve_at=8.)
        result = self.move(session, True)
        self.assertGreaterEqual(result['sim_time_s'], 8.)
        self.assertLess(result['sim_time_s'], 9.)
        self.assertEqual(session.preflights[0]['execution_budget']['simulation_budget_s'], 23.)
