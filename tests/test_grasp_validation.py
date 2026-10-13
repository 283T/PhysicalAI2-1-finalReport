import copy
import json
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_kinematics import axis_rotation, transform
from grasp_validation import (
    assess_hold,
    assess_opening,
    assess_preclose_pose,
    assess_relative_grasp,
    assess_seat,
    world_link_positions,
)


class GraspValidationTests(unittest.TestCase):
    def test_preclose_alignment_uses_camera_body_pose_and_rejects_offset_or_twist(self):
        body = transform([0.2, -0.1, 0.13], [0.12, -0.3, 0.7])
        relation = transform([0, -0.002, 0], [0.1, 0.2, 0.3])
        detection = {
            "center_xyz_m": body[:3, 3].tolist(),
            "rotation_world_from_connector": body[:3, :3].tolist(),
        }
        tool = body @ relation
        self.assertTrue(assess_preclose_pose(detection, tool, relation)["success"])
        displaced = tool.copy()
        displaced[0, 3] += 0.003
        self.assertFalse(assess_preclose_pose(detection, displaced, relation)["success"])
        twisted = tool @ transform(rpy=[0.04, 0, 0])
        self.assertFalse(assess_preclose_pose(detection, twisted, relation)["success"])
        invalid = tool.copy()
        invalid[:3, :3] = 0
        with self.assertRaises(ValueError):
            assess_preclose_pose(detection, invalid, relation)

    def test_opening_accepts_asymmetric_housing_but_rejects_empty_or_jammed_jaws(self):
        self.assertTrue(assess_opening([[0.002, 0.002], [0, 0.004]])["success"])
        self.assertFalse(assess_opening([[0, 0]])["success"])
        jammed = assess_opening([[0.006, 0.007]])
        self.assertFalse(jammed["success"])
        self.assertAlmostEqual(jammed["maximum_jaw_opening_m"], 0.029)

    def test_opening_rejects_missing_nonfinite_or_out_of_stroke_joint_values(self):
        for values in ([], [[0.002]], [[0.002, math.nan]], [[-0.002, 0.006]], [[0.03, 0]]):
            with self.subTest(values=values):
                self.assertFalse(assess_opening(values)["success"])

    def test_relative_grasp_allows_transport_rotation_but_rejects_local_slip(self):
        reference = {
            "tcp": [0, 0, 0],
            "plug": [0.02, 0, 0],
            "tcp_rotation_world": np.eye(3).tolist(),
            "plug_rotation_world": np.eye(3).tolist(),
        }
        r = axis_rotation([0, 0, 1], math.pi / 2)
        observation = {
            "tcp": [0.1, 0.2, 0.3],
            "plug": [0.1, 0.22, 0.3],
            "tcp_rotation_world": r.tolist(),
            "plug_rotation_world": r.tolist(),
        }
        self.assertTrue(assess_relative_grasp(reference, observation)["success"])
        observation["plug"][2] += 0.01
        self.assertFalse(assess_relative_grasp(reference, observation)["success"])
        observation["plug"][2] -= 0.01
        observation["plug_rotation_world"] = np.eye(3).tolist()
        self.assertFalse(assess_relative_grasp(reference, observation)["success"])
        observation.pop("plug_rotation_world")
        self.assertFalse(assess_relative_grasp(reference, observation)["success"])

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

    def test_orientation_change_is_checked_when_observed(self):
        closed = {**self.closed, "plug_rotation_world": np.eye(3).tolist()}
        for angle, success in ((0.02, True), (0.7, False)):
            rows = [
                {**row, "plug_rotation_world": axis_rotation([0, 1, 0], angle).tolist()}
                for row in self.rows
            ]
            result = assess_hold(self.baseline, rows, closed)
            self.assertEqual(result["success"], success)
            self.assertTrue(result["rotation_validated"])
            self.assertAlmostEqual(result["maximum_rotation_change_rad"], angle)

    def test_missing_or_invalid_orientation_cannot_pass_a_pose_hold(self):
        closed = {**self.closed, "plug_rotation_world": np.eye(3).tolist()}
        for matrix in (None, [[float("nan")] * 3] * 3, np.ones((3, 3)).tolist()):
            rows = [{**row, "plug_rotation_world": matrix} for row in self.rows]
            result = assess_hold(self.baseline, rows, closed)
            self.assertFalse(result["success"])
            self.assertIsNone(result["maximum_rotation_change_rad"])

    def test_seat_requires_position_orientation_and_stability_after_release(self):
        rows = [
            {"time_s": i / 10, "plug": [0, 0, 0.23], "plug_rotation_world": np.eye(3).tolist()}
            for i in range(12)
        ]
        self.assertTrue(assess_seat(rows)["success"])
        for key, value in (
            ("plug", [0, 0, 0.238]),
            ("plug", [0.003, 0, 0.23]),
            ("plug_rotation_world", axis_rotation([0, 1, 0], 0.15).tolist()),
            ("plug_rotation_world", None),
        ):
            with self.subTest(key=key, value=value):
                bad = copy.deepcopy(rows)
                bad[-1][key] = value
                self.assertFalse(assess_seat(bad)["success"])
        short = copy.deepcopy(rows)
        for row in short:
            row["time_s"] *= 0.1
        self.assertFalse(assess_seat(short)["success"])
        self.assertFalse(assess_seat(rows[:2])["success"])

    def test_reseat_rejects_motion_even_inside_position_tolerance(self):
        rows = [
            {
                "time_s": i / 10,
                "plug": [(-1) ** i * 0.0009, 0, 0.23],
                "plug_rotation_world": np.eye(3).tolist(),
            }
            for i in range(12)
        ]
        self.assertFalse(assess_seat(rows)["success"])


if __name__ == "__main__":
    unittest.main()
