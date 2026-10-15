import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from regrasp_plan import plan_top_pickup


class TopClosureScreenTest(unittest.TestCase):
    def test_open_approach_cannot_hide_a_collision_during_closure(self):
        class Bounds:
            def __init__(self, active):
                self.active = active
                self.joints = [
                    ET.fromstring(f'<joint name="right_arm_finger_{side}_joint" type="prismatic"/>')
                    for side in ("left", "right")
                ]

            def boxes(self, q, finger_intervals=None):
                closed = self.active and finger_intervals is not None
                center = [0, 0, 0.3] if closed else [2, 0, 0] if self.active else [4, 0, 0]
                return [("finger", (np.array(center), np.eye(3), np.full(3, 0.01)))]

        paths = {phase: {"right_arm": np.zeros((2, 4))} for phase in ("hover", "approach", "lift")}
        with tempfile.TemporaryDirectory() as directory:
            world = Path(directory) / "world.sdf"
            world.write_text(
                '<sdf><world><model name="fixtures"><static>true</static><link name="base"><collision name="stop"><pose>0 0 .3 0 0 0</pose><geometry><box><size>.1 .1 .1</size></box></geometry></collision></link></model><model name="conveyor_carrier"><static>true</static><pose>8 8 0 0 0 0</pose><link name="carrier"><collision name="pallet"><geometry><box><size>.1 .1 .1</size></box></geometry></collision></link></model></world></sdf>'
            )
            with patch("regrasp_plan.prepare_pickup_paths", return_value=(paths, {})):
                result = plan_top_pickup(
                    {"right_arm": object(), "left_arm": object()},
                    {"right_arm": Bounds(True), "left_arm": Bounds(False)},
                    world,
                    {"right_arm": np.zeros(4), "left_arm": np.zeros(4)},
                    "right_arm",
                    np.eye(4),
                )
            self.assertFalse(result["success"])
            self.assertTrue(all(a["fixtures"]["approach"]["clear"] for a in result["attempts"]))
            self.assertTrue(all(a["closure"]["fixtures"] is not None for a in result["attempts"]))


if __name__ == "__main__":
    unittest.main()
