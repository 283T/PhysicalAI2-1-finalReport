import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from assembly_case import make_case
from plan_reach import plan_reach
from test_assembly_case import prediction


class ReachInputTest(unittest.TestCase):
    def test_missing_duplicate_wrong_frame_and_nonfinite_observations_rejected(self):
        board = make_case(101, 2)["design"]
        plan = prediction(board)
        good = {
            "frame": "world",
            "sensor_stamp_ns": 1,
            "connectors": [
                {"plug_id": f"P{i}", "surface_xyz_m": [0, 0, 0.12], "yaw_rad": 0}
                for i in range(1, 4)
            ],
        }
        missing = copy.deepcopy(good)
        missing["connectors"].pop()
        duplicate = copy.deepcopy(good)
        duplicate["connectors"][1]["plug_id"] = "P1"
        frame = copy.deepcopy(good)
        frame["frame"] = "camera"
        invalid = copy.deepcopy(good)
        invalid["connectors"][0]["surface_xyz_m"][0] = float("nan")
        for observations in (missing, duplicate, frame, invalid):
            with self.assertRaises(ValueError):
                plan_reach(plan, board, observations, {})
