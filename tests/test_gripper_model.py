import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from gripper_model import OPEN_TRAVEL_M, add_gripper, gap_m


class GripperModelTest(unittest.TestCase):
    def test_two_opposed_mobile_fingers_have_contact_geometry_and_limits(self):
        robot = ET.Element("robot")
        add_gripper(robot, "test_", "flange")
        fingers = robot.findall("joint[@type='prismatic']")
        self.assertEqual(len(fingers), 2)
        self.assertEqual({joint.find("axis").get("xyz") for joint in fingers}, {"-1 0 0", "1 0 0"})
        for joint in fingers:
            self.assertEqual(joint.find("parent").get("link"), "test_gripper")
            link = robot.find(f"link[@name='{joint.find('child').get('link')}']")
            self.assertIsNotNone(link.find("collision/geometry/box"))
            self.assertGreater(float(joint.find("limit").get("upper")), OPEN_TRAVEL_M)
        self.assertAlmostEqual(gap_m(0), 0.016)
        self.assertAlmostEqual(gap_m(OPEN_TRAVEL_M), 0.052)
