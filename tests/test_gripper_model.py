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

    def test_split_pads_are_separated_inside_original_finger_envelope(self):
        robot = ET.Element("robot")
        add_gripper(robot, "test_", "flange", split_contacts=True)
        for name in ("left", "right"):
            pads = robot.findall(f"link[@name='test_finger_{name}']/collision")
            self.assertEqual(len(pads), 2)
            centers = [float(pad.find("origin").get("xyz").split()[1]) for pad in pads]
            self.assertAlmostEqual(centers[1] - centers[0], 0.012)
            for pad, center in zip(pads, centers):
                size = [float(v) for v in pad.find("geometry/box").get("size").split()]
                self.assertLessEqual(abs(center) + size[1] / 2, 0.01)


class FenceClearanceTest(unittest.TestCase):
    def test_shorter_fence_preserves_closed_overlap_and_increases_open_clearance(self):
        for width in (0.008, 0.012):
            robot = ET.Element("robot")
            add_gripper(
                robot,
                "test_",
                "flange",
                capture_fences=True,
                front_fence_side="right",
                fence_width_m=width,
            )
            for side, name in ((-1, "left"), (1, "right")):
                finger = robot.find(f"link[@name='test_finger_{name}']")
                rear = finger.find("collision[@name='rear_fence']")
                offset = float(rear.find("origin").get("xyz").split()[0])
                span = float(rear.find("geometry/box").get("size").split()[0])
                inner_closed = 0.012 + 0.002 + side * offset - span / 2
                inner_open = 0.012 + OPEN_TRAVEL_M + side * offset - span / 2
                self.assertLess(inner_closed, 0.010)
                self.assertAlmostEqual(inner_open, 0.030 - width)
                self.assertEqual(
                    finger.find("collision[@name='front_fence']") is not None, name == "right"
                )

    def test_invalid_fence_widths_are_rejected(self):
        for width in (float("nan"), float("inf"), 0.003, 0.013):
            with self.subTest(width=width), self.assertRaises(ValueError):
                add_gripper(ET.Element("robot"), "test_", "flange", fence_width_m=width)
