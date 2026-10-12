import math
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from tail_support_stage import ORIGIN_Z_M, TOP_Z_M, add_tail_stage, support_height


class TailSupportTests(unittest.TestCase):
    def test_support_stays_down_for_approach_and_accompanies_lift(self):
        self.assertEqual(support_height(0, -0.046, "open"), 0)
        self.assertEqual(support_height(0, -0.046, "approach"), 0)
        self.assertEqual(support_height(-0.046, -0.046, "close"), 0)
        self.assertAlmostEqual(support_height(0.054, -0.046, "lift"), 0.1)
        self.assertEqual(support_height(0.2, -0.046, "hold"), 0.12)
        self.assertEqual(support_height(0, -0.046, "retract"), 0)
        for value in (math.nan, math.inf):
            with self.assertRaises(ValueError):
                support_height(value, -0.046, "lift")

    def test_support_is_driven_by_world_joint_without_attaching_harness(self):
        world = ET.Element("world")
        stage = add_tail_stage(world)
        joint = stage.find("joint")
        self.assertEqual(joint.get("type"), "prismatic")
        self.assertEqual(joint.findtext("parent"), "world")
        self.assertEqual(joint.findtext("child"), "tray")
        size = [float(v) for v in stage.findtext("link/collision/geometry/box/size").split()]
        self.assertAlmostEqual(ORIGIN_Z_M + size[2] / 2, TOP_Z_M)
        center = float(stage.findtext("link/pose").split()[0])
        self.assertAlmostEqual(-0.07 - (center + size[0] / 2), 0.005)
        self.assertEqual(len(stage.findall("joint")), 1)
        self.assertIsNone(stage.find("static"))

    def test_static_control_has_identical_geometry_but_no_motion_joint(self):
        world = ET.Element("world")
        moving = add_tail_stage(world)
        fixed = add_tail_stage(ET.Element("world"), moving=False)
        self.assertEqual(ET.tostring(moving.find("link")), ET.tostring(fixed.find("link")))
        self.assertEqual(fixed.findtext("static"), "true")
        self.assertIsNone(fixed.find("joint"))
        self.assertIsNone(fixed.find("plugin"))


if __name__ == "__main__":
    unittest.main()
