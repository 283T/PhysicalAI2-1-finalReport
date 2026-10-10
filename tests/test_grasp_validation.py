import copy
import json
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from grasp_validation import assess_hold, world_link_positions


class GraspValidationTests(unittest.TestCase):
    def setUp(self):
        self.baseline = {"plug": [0, 0, 0.15]}
        self.closed = {"plug": [0, 0, 0.15], "tcp": [0, 0, 0.152]}
        self.rows = [
            {"plug": [0, 0, 0.25], "tcp": [0, 0, 0.252], "fingers": [0.002, 0.002]}
            for _ in range(12)
        ]

    def test_holding_moves_object_and_tool_together(self):
        result = assess_hold(self.baseline, self.rows, self.closed)
        self.assertTrue(result["success"])
        self.assertIs(json.loads(json.dumps(result))["success"], True)

    def test_off_center_housing_uses_total_opening(self):
        rows = copy.deepcopy(self.rows)
        for row in rows:
            row["fingers"] = [0.0003, 0.0037]
        self.assertTrue(assess_hold(self.baseline, rows, self.closed)["success"])

    def test_open_jaws_or_out_of_limit_fingers_are_not_a_pinch(self):
        for fingers in ([0.018, 0.018], [-0.01, 0.014]):
            rows = copy.deepcopy(self.rows)
            for row in rows:
                row["fingers"] = fingers
            self.assertFalse(assess_hold(self.baseline, rows, self.closed)["success"])

    def test_reject_stationary_object_empty_fingers_and_slip(self):
        for defect in (
            "stationary_object",
            "stationary_tool",
            "empty",
            "lifting_slip",
            "holding_slip",
        ):
            rows = copy.deepcopy(self.rows)
            for i, row in enumerate(rows):
                if defect == "stationary_object":
                    row["plug"][2] = 0.15
                elif defect == "stationary_tool":
                    row["tcp"][2] = 0.152
                elif defect == "empty":
                    row["fingers"] = [0, 0]
                elif defect == "lifting_slip":
                    row["plug"][2] -= 0.015
                elif defect == "holding_slip":
                    row["plug"][2] -= i * 0.001
            with self.subTest(defect=defect):
                self.assertFalse(assess_hold(self.baseline, rows, self.closed)["success"])
        self.assertFalse(assess_hold(self.baseline, self.rows[:3], self.closed)["success"])

    def test_moving_model_frame_is_not_assumed_constant(self):
        poses = {
            "harness": {
                "xyz": [1, 2, 3],
                "q": [0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)],
            },
            "terminal": {"xyz": [1, 0, 0]},
        }
        result = world_link_positions(poses, "harness", ["terminal"])
        np.testing.assert_allclose(result["terminal"], [1, 3, 3])
        poses["harness"]["xyz"][2] += 0.1
        result = world_link_positions(poses, "harness", ["terminal"])
        np.testing.assert_allclose(result["terminal"], [1, 3, 3.1])


if __name__ == "__main__":
    unittest.main()
