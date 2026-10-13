import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_kinematics import axis_rotation, transform
from preclose_alignment import correction_path, desired_tool_pose


class LocalArm:
    def forward(self, q):
        return transform(q[:3], [0, 0, q[3]])

    def inverse(self, target, seed, attempts):
        assert attempts == 1
        return {"success": True, "q_rad": [*target[:3, 3], np.arctan2(target[1, 0], target[0, 0])]}


class PrecloseAlignmentTests(unittest.TestCase):
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

    def test_reject_large_corrections(self):
        for goal in (transform([0.006, 0, 0]), transform([0, 0, 0], [0, 0, 0.11])):
            with self.assertRaises(ValueError):
                correction_path(LocalArm(), [0, 0, 0, 0], goal)

    def test_reject_ik_branch_jump_failure_and_nonfinite(self):
        for success, value in ((True, 3), (False, 0), (True, float("nan"))):

            class BrokenArm(LocalArm):
                def inverse(self, target, seed, attempts):
                    return {"success": success, "q_rad": [value, 0, 0, 0]}

            with self.subTest(success=success, value=value), self.assertRaises(ValueError):
                correction_path(BrokenArm(), [0, 0, 0, 0], transform([0.003, 0, 0]))
