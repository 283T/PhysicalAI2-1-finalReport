import copy
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from top_pickup_plan import plan_request


class TopPickupPlanTests(unittest.TestCase):
    def inputs(self):
        arms = {n: SimpleNamespace(names=list("abcdef")) for n in ("left_arm", "right_arm")}
        bounds = {
            n: SimpleNamespace(
                joints=[
                    ET.fromstring('<joint name="a" type="prismatic"/>'),
                    ET.fromstring('<joint name="b" type="prismatic"/>'),
                ]
            )
            for n in arms
        }
        request = {
            "primary_arm": "left_arm",
            "pose_source": "fresh_rgbd",
            "vision_connector_pose": np.eye(4).tolist(),
            "robot_feedback": {
                "sim_time_s": 1,
                "arms": {n: {"q_rad": [0] * 6, "fingers": [0.018] * 2} for n in arms},
            },
        }
        return arms, bounds, request

    def test_object_truth_extra_fields_and_closed_fingers_rejected(self):
        arms, bounds, original = self.inputs()
        for field, value in (("object_poses", {}), ("sim_time_s", float("nan"))):
            request = copy.deepcopy(original)
            request["robot_feedback"][field] = value
            with self.assertRaises(ValueError):
                plan_request(Path("unused"), arms, bounds, request)
        request = copy.deepcopy(original)
        request["robot_feedback"]["arms"]["left_arm"]["fingers"] = [0.006] * 2
        with self.assertRaises(ValueError):
            plan_request(Path("unused"), arms, bounds, request)

    def test_complete_stroke_envelope_is_forwarded_for_both_arms(self):
        arms, bounds, request = self.inputs()
        with patch("top_pickup_plan.plan_top_pickup", return_value={"success": False}) as worker:
            result = plan_request(Path("unused"), arms, bounds, request)
        self.assertFalse(result["success"])
        full = worker.call_args.args[1]
        for bound in full.values():
            self.assertEqual(bound.intervals, {"a": [-0.0005, 0.0205], "b": [-0.0005, 0.0205]})
