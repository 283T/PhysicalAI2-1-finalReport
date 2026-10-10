import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_kinematics import rotation_error, transform
from cooperative_transfer import assess_pair_path, paired_path, pose_samples, socket_poses


class PoseArm:
    def forward(self, q):
        return transform(q[:3], [0, 0, q[3]])

    def inverse(self, target, seed, attempts=1):
        yaw = math.atan2(target[1, 0], target[0, 0])
        yaw += round((seed[3] - yaw) / (2 * math.pi)) * 2 * math.pi
        return {"success": True, "q_rad": [*target[:3, 3], yaw]}


class TransferTests(unittest.TestCase):
    def test_half_turn_reaches_goal_with_small_rotation_steps(self):
        start, goal = transform([0, 0, 0]), transform([0.2, 0, 0], [math.pi, 0, 0])
        poses = pose_samples(start, goal)
        np.testing.assert_allclose(poses[0], start, atol=1e-10)
        np.testing.assert_allclose(poses[-1], goal, atol=1e-10)
        for first, second in zip(poses, poses[1:]):
            self.assertLessEqual(
                np.linalg.norm(rotation_error(second[:3, :3], first[:3, :3])), 0.0251
            )

    def test_pair_rotates_about_primary_preserving_relative_frames(self):
        arms = {"a": PoseArm(), "b": PoseArm()}
        state = {"a": [0, 0, 0.3, 0], "b": [0.3, 0, 0.3, 0]}
        paths = paired_path(arms, state, "a", transform([0.4, 0.2, 0.3], [0, 0, math.pi / 2]))
        np.testing.assert_allclose(paths["b"][-1][:3], [0.4, 0.5, 0.3], atol=1e-6)
        result = assess_pair_path(arms, paths)
        self.assertTrue(result["success"])
        self.assertLess(result["maximum_relative_position_error_m"], 0.0001)

    def test_reject_unreachable_support_and_joint_jump(self):
        class BrokenArm(PoseArm):
            def inverse(self, target, seed, attempts=1):
                return {"success": True, "q_rad": [5, 0, 0, 0]}

        class UnreachableArm(PoseArm):
            def inverse(self, target, seed, attempts=1):
                return {"success": False, "q_rad": seed}

        for arm in (BrokenArm(), UnreachableArm()):
            with self.subTest(arm=type(arm).__name__), self.assertRaises(ValueError):
                paired_path(
                    {"a": PoseArm(), "b": arm},
                    {"a": [0, 0, 0, 0], "b": [0.3, 0, 0, 0]},
                    "a",
                    transform([0.1, 0, 0]),
                )

    def test_orbit_preserves_distance_while_allowing_relative_orientation_change(self):
        arms = {"a": PoseArm(), "b": PoseArm()}
        state = {"a": [0, 0, 0.3, 0], "b": [0.3, 0, 0.3, 0]}
        paths = paired_path(
            arms,
            state,
            "a",
            transform([0.4, 0.2, 0.3], [0, 0, math.pi / 2]),
            support_orbit=-math.pi / 2,
        )
        np.testing.assert_allclose(paths["b"][-1], [0.4, -0.1, 0.3, 0], atol=1e-6)
        self.assertTrue(assess_pair_path(arms, paths, rigid=False)["success"])
        self.assertFalse(assess_pair_path(arms, paths, rigid=True)["success"])

    def test_pair_assessment_rejects_missing_or_nonfinite_paths(self):
        arms = {"a": PoseArm(), "b": PoseArm()}
        for paths in (
            {"a": [[0, 0, 0, 0]]},
            {"a": [[0, 0, 0, 0]] * 2, "b": [[float("nan"), 0, 0, 0]] * 2},
        ):
            with self.subTest(paths=paths), self.assertRaises(ValueError):
                assess_pair_path(arms, paths)

    def test_pair_error_detects_support_left_at_pallet(self):
        result = assess_pair_path(
            {"a": PoseArm(), "b": PoseArm()},
            {
                "a": np.array([[0, 0, 0, 0], [0.1, 0, 0, 0]]),
                "b": np.array([[0.3, 0, 0, 0], [0.3, 0, 0, 0]]),
            },
        )
        self.assertFalse(result["success"])
        self.assertAlmostEqual(result["maximum_relative_position_error_m"], 0.1)

    def test_seated_height_and_grasp_relation(self):
        relation = transform([0.01, 0.02, 0.003], [math.pi, 0, 0])
        poses = socket_poses(
            {"origin_m": [0.16, 0.08, 0.14]}, {"x_mm": 50, "y_mm": 40, "yaw_deg": 90}, relation
        )
        connector = poses["seated"] @ np.linalg.inv(relation)
        np.testing.assert_allclose(connector[:3, 3], [0.21, 0.12, 0.15])
        self.assertAlmostEqual((poses["hover"] @ np.linalg.inv(relation))[2, 3], 0.30)

    def test_reject_bad_pose_sampling_limits_and_hover(self):
        bad = np.eye(4)
        bad[0, 0] = 2
        with self.assertRaises(ValueError):
            pose_samples(np.eye(4), bad)
        for value in (0, -1, float("nan")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                pose_samples(np.eye(4), np.eye(4), spacing=value)
        with self.assertRaises(ValueError):
            socket_poses(
                {"origin_m": [0, 0, 0]},
                {"x_mm": 0, "y_mm": 0, "yaw_deg": 0},
                np.eye(4),
                hover_height=0.01,
            )


if __name__ == "__main__":
    unittest.main()
