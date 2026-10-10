import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_kinematics import transform
from cartesian_path import interpolate_joints, translation_path


class CartesianArm:
    def forward(self, q):
        return transform(q[:3])

    def inverse(self, target, seed):
        return {"success": True, "q_rad": target[:3, 3]}


class CartesianPathTests(unittest.TestCase):
    def test_straight_lift_retains_horizontal_position(self):
        path = translation_path(CartesianArm(), [0.1, 0.2, 0.15], transform([0.1, 0.2, 0.25]))
        np.testing.assert_allclose(path[:, :2], np.tile([0.1, 0.2], (len(path), 1)))
        np.testing.assert_allclose(path[-1], [0.1, 0.2, 0.25])
        self.assertTrue(np.all(np.diff(path[:, 2]) > 0))

    def test_reject_unreachable_waypoint_before_returning_a_path(self):
        class LimitedArm(CartesianArm):
            def inverse(self, target, seed):
                return {"success": target[2, 3] < 0.2, "q_rad": target[:3, 3]}

        with self.assertRaises(ValueError):
            translation_path(LimitedArm(), [0.1, 0.2, 0.15], transform([0.1, 0.2, 0.25]))

    def test_reject_discontinuous_or_nonfinite_ik(self):
        for value in (5.0, float("nan")):

            class BrokenArm(CartesianArm):
                def inverse(self, target, seed):
                    return {"success": True, "q_rad": [value, 0, 0]}

            with self.subTest(value=value), self.assertRaises(ValueError):
                translation_path(BrokenArm(), [0, 0, 0], transform([0, 0, 0.1]))

    def test_translation_requires_aligned_tool(self):
        with self.assertRaises(ValueError):
            translation_path(CartesianArm(), [0, 0, 0], transform([0, 0, 0.1], [0, 0.2, 0]))

    def test_interpolation_is_bounded_and_reaches_endpoints(self):
        path = np.array([[0, 0], [1, 2], [2, 4]])
        np.testing.assert_allclose(interpolate_joints(path, -0.1), [0, 0])
        np.testing.assert_allclose(interpolate_joints(path, 0.25), [0.5, 1])
        np.testing.assert_allclose(interpolate_joints(path, 1.1), [2, 4])


if __name__ == "__main__":
    unittest.main()
