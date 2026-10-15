import copy
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from assembly_case import make_case
from top_socket_plan import plan_socket


class TopSocketPlanTests(unittest.TestCase):
    def inputs(self):
        board = make_case(101, 2)["design"]
        sockets = {s["id"]: s for s in board["sockets"]}
        drawing = {k: board[k] for k in ["drawing_id", "board_id", "harness_id"]}
        drawing["connections"] = [
            {**r, **{k: sockets[r["socket"]][k] for k in ["x_mm", "y_mm", "yaw_deg"]}}
            for r in board["connections"]
        ]
        arm = Mock()
        arm.base = np.eye(4)
        arm.limits = np.tile([-3, 3], (6, 1))
        arm.inverse.return_value = {"success": True, "q_rad": [0] * 6}
        bound = Mock()
        bound.joints = [
            ET.fromstring('<joint type="prismatic" name="finger_left_joint"/>'),
            ET.fromstring('<joint type="prismatic" name="finger_right_joint"/>'),
        ]
        return (
            board,
            drawing,
            {"left": arm, "right": arm},
            {"left": bound, "right": bound},
            {"left": np.zeros(6), "right": np.zeros(6)},
        )

    def test_arm_clearance_cannot_override_a_carried_pin_obstruction(self):
        board, drawing, arms, bounds, current = self.inputs()
        with (
            patch("top_socket_plan.FixtureBounds") as fixtures,
            patch("top_socket_plan.CarriedConnector") as carried,
            patch(
                "top_socket_plan.joint_route",
                return_value={"success": True, "path": [[0] * 6, [0] * 6]},
            ),
            patch("top_socket_plan.precise_path", return_value=np.zeros((2, 6))),
            patch("top_socket_plan.screen_paths", return_value={"clear": True}),
        ):
            fixtures.return_value.screen.return_value = {"clear": True}
            carried.return_value.screen.return_value = {
                "clear": False,
                "overlap": {"connector_shape": "pin_0_collision", "fixture": "obstacle"},
            }
            report = plan_socket(
                arms, bounds, Path("unused.sdf"), current, "left", board, drawing, "P3", np.eye(4)
            )
        self.assertFalse(report["success"])
        self.assertNotIn("paths", report)
        self.assertTrue(all(not a["success"] for a in report["attempts"]))

    def test_drawing_socket_cannot_be_replaced_with_a_more_convenient_target(self):
        board, drawing, arms, bounds, current = self.inputs()
        wrong = copy.deepcopy(drawing)
        wrong["connections"][-1]["x_mm"] += 1
        with patch("top_socket_plan.CarriedConnector") as carried:
            with self.assertRaises(ValueError):
                plan_socket(
                    arms, bounds, Path("unused.sdf"), current, "left", board, wrong, "P3", np.eye(4)
                )
            carried.assert_not_called()
