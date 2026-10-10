import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_collision_bounds import intersects, screen_paths


class BoundsTests(unittest.TestCase):
    def test_margin_rejects_nearly_touching_boxes(self):
        a = (np.zeros(3), np.eye(3), np.ones(3) * 0.1)
        b = (np.array([0.209, 0, 0]), np.eye(3), np.ones(3) * 0.1)
        self.assertFalse(intersects(a, b, margin=0))
        self.assertTrue(intersects(a, b, margin=0.005))
        b = (np.array([0.211, 0, 0]), np.eye(3), np.ones(3) * 0.1)
        self.assertFalse(intersects(a, b, margin=0.005))

    def test_rotated_parallel_rods_with_overlapping_axis_aligned_boxes(self):
        angle = math.pi / 4
        rotation = np.array(
            [
                [math.cos(angle), -math.sin(angle), 0],
                [math.sin(angle), math.cos(angle), 0],
                [0, 0, 1],
            ]
        )
        a = (np.zeros(3), rotation, np.array([1, 0.1, 0.1]))
        b = (np.array([-0.22, 0.22, 0]), rotation, np.array([1, 0.1, 0.1]))
        self.assertFalse(intersects(a, b))
        b = (np.array([-0.1, 0.1, 0]), rotation, np.array([1, 0.1, 0.1]))
        self.assertTrue(intersects(a, b))

    def test_crossing_paths_are_rejected_before_endpoint(self):
        class BoxArm:
            def boxes(self, q):
                return [("body", (np.asarray(q), np.eye(3), np.ones(3) * 0.02))]

        bounds = {"left_arm": BoxArm(), "right_arm": BoxArm()}
        paths = {
            "left_arm": np.array([[0, 0, 0], [0.4, 0, 0]]),
            "right_arm": np.array([[0.4, 0, 0], [0, 0, 0]]),
        }
        result = screen_paths(bounds, paths)
        self.assertFalse(result["clear"])
        self.assertLess(result["fraction"], 1)
        self.assertEqual(result["links"], ["body", "body"])


if __name__ == "__main__":
    unittest.main()
