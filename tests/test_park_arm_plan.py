import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from park_arm_plan import parking_plan


class ParkArmPlanTests(unittest.TestCase):
    def inputs(self):
        arm = Mock()
        arm.names = list("abcdef")
        arm.limits = np.tile([-3, 3], (6, 1))
        bound = Mock()
        bound.joints = [
            ET.fromstring('<joint type="prismatic" name="finger_left"/>'),
            ET.fromstring('<joint type="prismatic" name="finger_right"/>'),
        ]
        row = {
            "sim_time_s": 1,
            "arms": {n: {"q_rad": [0] * 6, "fingers": [0.018, 0.018]} for n in ["left", "right"]},
        }
        return {"left": arm, "right": arm}, {"left": bound, "right": bound}, row

    def test_closed_fingers_and_object_truth_are_not_parking_feedback(self):
        arms, bounds, row = self.inputs()
        row["arms"]["left"]["fingers"] = [0.006, 0.006]
        with self.assertRaises(ValueError):
            parking_plan(Path("unused"), arms, bounds, row, "left")
        row["arms"]["left"]["fingers"] = [0.018, 0.018]
        row["evaluation_poses"] = {"P3": [1, 2, 3]}
        with self.assertRaises(ValueError):
            parking_plan(Path("unused"), arms, bounds, row, "left")

    def test_route_success_cannot_hide_a_fixture_sweep_failure(self):
        arms, bounds, row = self.inputs()
        with (
            patch("park_arm_plan.FixtureBounds") as fixtures,
            patch(
                "park_arm_plan.joint_route",
                return_value={"success": True, "path": [[0] * 6, [0] * 6]},
            ),
            patch("park_arm_plan.screen_paths", return_value={"clear": True}),
        ):
            fixtures.return_value.screen.return_value = {
                "clear": False,
                "shapes": ["finger", "fixture"],
            }
            result = parking_plan(Path("unused"), arms, bounds, row, "left")
        self.assertFalse(result["success"])
        self.assertNotIn("paths", result)
