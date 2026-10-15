import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from socket_plan_worker import plan_request


class SocketWorkerTests(unittest.TestCase):
    def request(self):
        return {
            "primary_arm": "left_arm",
            "plug_id": "P3",
            "pose_source": "fresh_rgbd",
            "vision_connector_from_tool": np.eye(4).tolist(),
            "robot_feedback": {
                "sim_time_s": 10,
                "arms": {
                    n: {"q_rad": [0] * 6, "fingers": [0.007] * 2} for n in ("left_arm", "right_arm")
                },
            },
        }

    def test_loaded_robot_feedback_valid_but_no_object_truth(self):
        arms = {n: SimpleNamespace(names=list("abcdef")) for n in ("left_arm", "right_arm")}
        request = self.request()
        with patch("socket_plan_worker.plan_socket", return_value={"success": False}) as planner:
            result = plan_request(Path("unused"), arms, {}, {}, {}, request)
        self.assertFalse(result["success"])
        planner.assert_called_once()
        request["robot_feedback"]["arms"]["left_arm"]["plug"] = [0, 0, 0]
        with self.assertRaises(ValueError):
            plan_request(Path("unused"), arms, {}, {}, {}, request)

    def test_cached_or_nominal_relation_not_accepted_as_fresh(self):
        request = self.request()
        request["pose_source"] = "nominal"
        with self.assertRaises(ValueError):
            plan_request(Path("unused"), {}, {}, {}, {}, request)
