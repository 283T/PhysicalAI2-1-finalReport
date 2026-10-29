import json
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_kinematics import axis_rotation, transform
from preclose_alignment import (
    correction_budget,
    correction_path,
    correction_tracking,
    desired_tool_pose,
    outward_grasp_clearance,
    tracking_compensated_goal,
)


class LocalArm:
    def forward(self, q):
        return transform(q[:3], [0, 0, q[3]])

    def inverse(self, target, seed, attempts, **tolerances):
        assert attempts == 1
        return {"success": True, "q_rad": [*target[:3, 3], np.arctan2(target[1, 0], target[0, 0])]}


class PrecloseAlignmentTests(unittest.TestCase):
    def test_clearance_uses_the_tool_axis_and_preserves_rotation_and_input(self):
        grasp = transform([.1, .2, .3], [.3, -.4, .8])
        original = grasp.copy()
        shifted = outward_grasp_clearance(grasp, .002)
        np.testing.assert_allclose(shifted[:3, 3], grasp[:3, 3] - .002 * grasp[:3, 2])
        np.testing.assert_array_equal(shifted[:3, :3], grasp[:3, :3])
        np.testing.assert_array_equal(grasp, original)
        np.testing.assert_array_equal(outward_grasp_clearance(grasp, 0), grasp)

    def test_invalid_grasp_clearance_is_rejected(self):
        for distance in [-.001, .0041, float('nan'), float('inf')]:
            with self.subTest(distance=distance), self.assertRaises(ValueError):
                outward_grasp_clearance(np.eye(4), distance)

    def test_visual_servo_compensates_a_fixed_tool_tracking_bias(self):
        bias = transform([0.0008, -0.0004, 0], [0, 0, 0.008])
        commanded = transform([0.1, 0.2, 0.3], [0, 0, 0.1])
        observed = commanded @ bias
        desired = transform([0.101, 0.2, 0.3], [0, 0, 0.11])
        corrected = tracking_compensated_goal(desired, observed, commanded)
        # A plant with the same fixed hand-frame bias must now reach the target.
        np.testing.assert_allclose(corrected @ bias, desired, atol=1e-10)

    def test_image_frame_offset_is_rotated_into_world(self):
        body_rotation = axis_rotation([0, 0, 1], np.pi / 2)
        pose = desired_tool_pose(
            {"center_xyz_m": [1, 2, 3], "rotation_world_from_connector": body_rotation},
            transform([0.002, 0, 0]),
        )
        np.testing.assert_allclose(pose[:3, 3], [1, 2.002, 3])

    def test_small_translation_and_rotation_follow_continuous_branch(self):
        goal = transform([0.003, 0, 0], [0, 0, 0.048])
        path = correction_path(LocalArm(), [0, 0, 0, 0], goal)
        np.testing.assert_allclose(LocalArm().forward(path[-1]), goal, atol=1e-8)
        self.assertLessEqual(np.max(np.abs(np.diff(path, axis=0))), 0.011)

    def test_retries_share_translation_and_rotation_budget(self):
        first = {"arm": transform([0.003, 0, 0], [0, 0, 0.04])}
        used = correction_budget({}, {"arm": np.eye(4)}, first)
        second = {"arm": transform([0.002, 0, 0], [0, 0, 0.01])}
        used = correction_budget(used, first, second)
        self.assertAlmostEqual(used["arm"]["translation_m"], 0.004)
        self.assertAlmostEqual(used["arm"]["rotation_rad"], 0.07)
        # Returning to the starting pose still consumes distance; retries cannot reset it.
        with self.assertRaises(ValueError):
            correction_budget(used, second, {"arm": np.eye(4)})
        with self.assertRaises(ValueError):
            correction_budget(used, second, {"arm": transform([0.002, 0, 0], [0, 0, 0.05])})

    def test_tracking_failure_is_distinct_from_permission_to_reinspect(self):
        result = correction_tracking(0.00087, 0.00554, 0.00556, 0.000001)
        self.assertTrue(result["within_move_limits"])
        self.assertFalse(result["precision_reached"])
        self.assertTrue(correction_tracking(0.0002, 0.002, 0.001, 0.000001)["precision_reached"])
        for index, value in enumerate((0.002, 0.025, 0.01, 0.001)):
            errors = [0.0001, 0.001, 0.0001, 0.00001]
            errors[index] = value
            with self.subTest(index=index):
                self.assertFalse(correction_tracking(*errors)["within_move_limits"])
        self.assertFalse(correction_tracking(float("nan"), 0, 0, 0)["within_move_limits"])

    def test_numpy_tracking_values_serialize_without_losing_failed_precision(self):
        values = np.array([0.00087, 0.00554, 0.00556, 0.000001])
        result = json.loads(json.dumps(correction_tracking(*values)))
        self.assertIs(result["within_move_limits"], True)
        self.assertIs(result["precision_reached"], False)

    def test_reject_large_corrections(self):
        for goal in (transform([0.006, 0, 0]), transform([0, 0, 0], [0, 0, 0.11])):
            with self.assertRaises(ValueError):
                correction_path(LocalArm(), [0, 0, 0, 0], goal)

    def test_reject_ik_branch_jump_failure_and_nonfinite(self):
        for success, value in ((True, 3), (False, 0), (True, float("nan"))):

            class BrokenArm(LocalArm):
                def inverse(self, target, seed, attempts, **tolerances):
                    return {"success": success, "q_rad": [value, 0, 0, 0]}

            with self.subTest(success=success, value=value), self.assertRaises(ValueError):
                correction_path(BrokenArm(), [0, 0, 0, 0], transform([0.003, 0, 0]))
