"""Initial high-resolution refinement must precede motion and retain grasp geometry."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import dual_pickup
from arm_kinematics import transform


class Planned(Exception):
    pass


class InitialInspectionTests(unittest.TestCase):
    def setUp(self):
        self.body = transform([.1, .2, .3], [.2, -.1, .5])
        self.relation = transform([.01, -.02, .03], [.1, .4, -.2])
        self.fine = transform([.101, .199, .302], [.21, -.11, .49])

        def observation(body, stamp, grasp=False):
            pose = {'center_xyz_m': body[:3, 3].tolist(),
                    'rotation_world_from_connector': body[:3, :3].tolist()}
            if grasp:
                pose['grasp_tcp_pose'] = (body @ self.relation).tolist()
            return {'connectors': [{'plug_id': 'P3', 'pose_6d': pose}],
                    'sensor_stamp_ns': stamp, 'stable_frames': 3}

        self.overview = observation(self.body, 1000000000, True)
        self.inspection = observation(self.fine, 4000000000)
        self.session = SimpleNamespace(
            snapshot=Mock(return_value={'arms': {n: {'q_rad': [0.] * 6}
                                                for n in ['left_arm', 'right_arm']}}),
            arms={'right_arm': Mock()}, bounds=Mock(), move=Mock())

    def run_to_planning(self, enabled, observations, clearance=0):
        report = {}
        with patch('dual_pickup.acquire_observation', side_effect=observations) as acquire, \
             patch('dual_pickup.prepare_pickup_paths', side_effect=Planned) as plan:
            with self.assertRaises(Planned):
                dual_pickup.primary_pickup(
                    self.session, Path('/tmp/test-pickup'), Path('/tmp/calibration.json'),
                    report, Mock(), Mock(), holding_args=SimpleNamespace(
                        refine_initial_with_inspection=enabled, primary_grasp_clearance_m=clearance))
        self.session.move.assert_not_called()
        return report, acquire, plan

    def test_refined_goal_is_used_before_any_motion_with_the_same_grasp_relation(self):
        report, acquire, plan = self.run_to_planning(True, [self.overview, self.inspection])
        expected = self.fine @ self.relation
        np.testing.assert_allclose(plan.call_args.args[3]['right_arm'], expected, atol=1e-12)
        hover = plan.call_args.args[2]['right_arm']
        np.testing.assert_allclose(hover[:3, 3], expected[:3, 3] - .1 * expected[:3, 2])
        self.assertEqual(acquire.call_args.kwargs['calibration'], Path('/tmp/calibration.json'))
        self.assertEqual(report['initial_inspection_refinement']['inspection_sensor_stamp_ns'],
                         4000000000)

    def test_clearance_is_preserved_in_the_refined_connector_frame(self):
        _, _, plan = self.run_to_planning(True, [self.overview, self.inspection], .002)
        shifted = self.body @ self.relation
        shifted[:3, 3] -= .002 * shifted[:3, 2]
        expected = self.fine @ np.linalg.inv(self.body) @ shifted
        np.testing.assert_allclose(plan.call_args.args[3]['right_arm'], expected, atol=1e-12)

    def test_default_keeps_the_original_overview_goal(self):
        report, acquire, plan = self.run_to_planning(False, [self.overview])
        np.testing.assert_allclose(plan.call_args.args[3]['right_arm'], self.body @ self.relation)
        acquire.assert_called_once()
        self.assertNotIn('initial_inspection_refinement', report)

    def test_failed_inspection_never_falls_back_to_a_coarse_motion(self):
        with patch('dual_pickup.acquire_observation', side_effect=[
                self.overview, RuntimeError('Fresh observation rejected')]), \
             patch('dual_pickup.prepare_pickup_paths') as plan:
            with self.assertRaisesRegex(RuntimeError, 'Fresh observation rejected'):
                dual_pickup.primary_pickup(
                    self.session, Path('/tmp/test-pickup'), Path('/tmp/calibration.json'),
                    {}, Mock(), Mock(), holding_args=SimpleNamespace(
                        refine_initial_with_inspection=True))
        plan.assert_not_called()
        self.session.move.assert_not_called()

    def test_mismatched_identity_cannot_change_the_target_connector(self):
        self.inspection['connectors'][0]['plug_id'] = 'P2'
        with patch('dual_pickup.acquire_observation', side_effect=[self.overview, self.inspection]), \
             patch('dual_pickup.prepare_pickup_paths') as plan:
            with self.assertRaisesRegex(RuntimeError, 'identity mismatch'):
                dual_pickup.primary_pickup(
                    self.session, Path('/tmp/test-pickup'), Path('/tmp/calibration.json'),
                    {}, Mock(), Mock(), holding_args=SimpleNamespace(
                        refine_initial_with_inspection=True))
        plan.assert_not_called()
        self.session.move.assert_not_called()
