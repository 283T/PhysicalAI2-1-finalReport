import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_collision_bounds import ArmCollisionBounds, intersects, screen_paths, with_parked_arms


class BoundsTests(unittest.TestCase):
    def test_measured_interval_encloses_every_finger_position_in_that_interval(self):
        source = '<robot><link name="world"/><link name="finger"><collision><origin xyz="0 0 .04" rpy="0 0 .4"/><geometry><box size=".008 .02 .08"/></geometry></collision></link><joint name="jaw" type="prismatic"><parent link="world"/><child link="finger"/><origin xyz=".012 0 0"/><axis xyz="1 0 0"/></joint></robot>'

        class Arm:
            names = []
            base = np.eye(4)

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "robot.urdf"
            path.write_text(source)
            bounds = ArmCollisionBounds(path, Arm())
            _, (center, axes, half) = bounds.boxes([], finger_intervals={"jaw": [0.0175, 0.0185]})[
                0
            ]
            for travel in np.linspace(0.0175, 0.0185, 11):
                _, (point, rotation, actual_half) = bounds.boxes(
                    [], finger_intervals={"jaw": [travel, travel]}
                )[0]
                for signs in np.ndindex(2, 2, 2):
                    corner = point + rotation @ ((np.array(signs) * 2 - 1) * actual_half)
                    self.assertTrue(np.all(np.abs(axes.T @ (corner - center)) <= half + 1e-10))
            for interval in ([0.019, 0.018], [0, 0.03], [0, float("nan")]):
                with self.assertRaises(ValueError):
                    bounds.boxes([], finger_intervals={"jaw": interval})

    def test_single_arm_path_is_checked_against_observed_parked_arm(self):
        class BoxArm:
            def boxes(self, q):
                return [("body", (np.asarray(q), np.eye(3), np.ones(3) * 0.02))]

        active = {"right": np.array([[0, 0, 0], [0.4, 0, 0]])}
        paths = with_parked_arms(active, {"left": [0.2, 0, 0]})
        self.assertFalse(screen_paths({"left": BoxArm(), "right": BoxArm()}, paths)["clear"])
        paths = with_parked_arms(active, {"left": [0.2, 0.2, 0]})
        self.assertTrue(screen_paths({"left": BoxArm(), "right": BoxArm()}, paths)["clear"])
        np.testing.assert_array_equal(paths["left"], [[0.2, 0.2, 0], [0.2, 0.2, 0]])
        with self.assertRaises(ValueError):
            with_parked_arms(active, {"right": [0, 0, 0]})

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
