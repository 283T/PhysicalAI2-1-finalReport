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


class HousingEndStopTest(unittest.TestCase):
    def test_end_stops_leave_marker_and_cable_clear_with_small_housing_end_gap(self):
        robot = ET.Element("robot")
        add_gripper(
            robot,
            "test_",
            "flange",
            split_contacts=True,
            capture_fences=True,
            front_fence_side="right",
            fence_width_m=0.008,
            housing_end_stops=True,
        )
        finger = robot.find("link[@name='test_finger_right']")
        self.assertIsNone(
            robot.find("link[@name='test_finger_left']/collision[@name='housing_end_plus']")
        )
        for name in ("housing_end_minus", "housing_end_plus"):
            shape = finger.find(f"collision[@name='{name}']")
            center = [float(v) for v in shape.find("origin").get("xyz").split()]
            size = [float(v) for v in shape.find("geometry/box").get("size").split()]
            inner_x = 0.012 + 0.002 + center[0] - size[0] / 2
            self.assertGreater(inner_x, 0.008)  # Half of the visual marker is 8 mm.
            self.assertGreater(inner_x, 0.004)  # Half of the cable cross section is 4 mm.
            self.assertAlmostEqual(abs(center[1]) - size[1] / 2 - 0.020, 0.0003)
        centers = [
            abs(float(p.find("origin").get("xyz").split()[1]))
            for p in finger.findall("collision")
            if p.get("name", "").startswith("contact_pad")
        ]
        self.assertTrue(all(0.006 < value < 0.010 for value in centers))
        self.assertGreater(float(finger.find("inertial/mass").get("value")), 0.012)
        with self.assertRaises(ValueError):
            add_gripper(ET.Element("robot"), "test_", "flange", housing_end_stops=True)


class PinSideClearanceTest(unittest.TestCase):
    def test_pin_side_pads_and_visual_end_before_nominal_tcp(self):
        robot = ET.Element("robot")
        add_gripper(
            robot,
            "test_",
            "flange",
            split_contacts=True,
            capture_fences=True,
            front_fence_side="right",
            housing_end_stops=True,
            pin_side_tip_recess_m=0.002,
        )
        for name, end in (("left", 0.078), ("right", 0.080)):
            finger = robot.find(f"link[@name='test_finger_{name}']")
            shapes = [
                finger.find("visual"),
                *[
                    p
                    for p in finger.findall("collision")
                    if p.get("name", "").startswith("contact_pad")
                ],
            ]
            for shape in shapes:
                z = float(shape.find("origin").get("xyz").split()[2])
                length = float(shape.find("geometry/box").get("size").split()[2])
                self.assertAlmostEqual(z + length / 2, end)
        for value in (-0.001, 0.004, float("nan")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                add_gripper(ET.Element("robot"), "test_", "flange", pin_side_tip_recess_m=value)


class EndClearanceTest(unittest.TestCase):
    def test_custom_clearance_keeps_finite_physical_gap(self):
        robot = ET.Element("robot")
        add_gripper(
            robot,
            "test_",
            "flange",
            capture_fences=True,
            housing_end_stops=True,
            housing_end_clearance_m=0.0001,
        )
        for sign, name in ((-1, "minus"), (1, "plus")):
            shape = robot.find(
                f"link[@name='test_finger_right']/collision[@name='housing_end_{name}']"
            )
            y = float(shape.find("origin").get("xyz").split()[1])
            width = float(shape.find("geometry/box").get("size").split()[1])
            self.assertAlmostEqual(sign * y - width / 2 - 0.020, 0.0001)
        for clearance in (0, 0.002, float("nan")):
            with self.subTest(clearance=clearance), self.assertRaises(ValueError):
                add_gripper(
                    ET.Element("robot"),
                    "test_",
                    "flange",
                    capture_fences=True,
                    housing_end_stops=True,
                    housing_end_clearance_m=clearance,
                )
        with self.assertRaises(ValueError):
            add_gripper(ET.Element("robot"), "test_", "flange", housing_end_clearance_m=0.0001)
