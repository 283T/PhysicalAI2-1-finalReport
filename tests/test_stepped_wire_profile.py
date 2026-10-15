import math
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from gripper_model import add_gripper
from stepped_wire_profile import stepped_profile


class SteppedWireProfileTests(unittest.TestCase):
    def source(self):
        robot = ET.Element("robot")
        add_gripper(
            robot,
            "test_",
            "root",
            split_contacts=True,
            pad_inward_extension_m=0.005,
            wire_contact_length_m=0.050,
        )
        return robot

    def test_narrow_tip_broad_shoulder_and_inertia_use_same_solid_geometry(self):
        source = self.source()
        before = ET.tostring(source)
        candidate = stepped_profile(source, "test_")
        self.assertEqual(ET.tostring(source), before)
        for name in ("left", "right"):
            old = source.find(f"link[@name='test_finger_{name}']")
            finger = candidate.find(f"link[@name='test_finger_{name}']")
            pads = finger.findall("collision")
            self.assertEqual(len(pads), 4)
            pieces = [
                (
                    tuple(map(float, p.find("geometry/box").get("size").split())),
                    tuple(map(float, p.find("origin").get("xyz").split())),
                )
                for p in pads
            ]
            for size, xyz in pieces:
                low, high = xyz[2] - size[2] / 2, xyz[2] + size[2] / 2
                if high < 0.079:
                    self.assertAlmostEqual(low, 0)
                    self.assertAlmostEqual(high, 0.067)
                    self.assertAlmostEqual(abs(xyz[1]) + size[1] / 2, 0.025)
                else:
                    self.assertAlmostEqual(low, 0.067)
                    self.assertAlmostEqual(high, 0.080)
                    self.assertAlmostEqual(abs(xyz[1]) + size[1] / 2, 0.010)
            mass = float(finger.find("inertial/mass").get("value"))
            density = float(old.find("inertial/mass").get("value")) / (2 * 0.013 * 0.020 * 0.080)
            self.assertAlmostEqual(mass, density * sum(math.prod(size) for size, _ in pieces))
            center = tuple(map(float, finger.find("inertial/origin").get("xyz").split()))
            self.assertAlmostEqual(center[1], 0)
            self.assertAlmostEqual(
                center[2], sum(density * math.prod(size) * xyz[2] for size, xyz in pieces) / mass
            )
            inertia = finger.find("inertial/inertia")
            expected = sum(
                density
                * math.prod(size)
                * ((size[1] ** 2 + size[2] ** 2) / 12 + xyz[1] ** 2 + (xyz[2] - center[2]) ** 2)
                for size, xyz in pieces
            )
            self.assertAlmostEqual(float(inertia.get("ixx")), expected)
            self.assertGreater(float(inertia.get("iyy")), 0)
            self.assertGreater(float(inertia.get("izz")), 0)

    def test_unsupported_profile_and_nonfinite_dimensions_rejected(self):
        for shoulder in (0, 0.079, math.nan, math.inf):
            with self.assertRaises(ValueError):
                stepped_profile(self.source(), "test_", shoulder_z=shoulder)
        for span in (0, 0.050, math.nan):
            with self.assertRaises(ValueError):
                stepped_profile(self.source(), "test_", tip_span=span)
        robot = self.source()
        robot.find("link[@name='test_finger_left']/collision").set("name", "fence")
        with self.assertRaises(ValueError):
            stepped_profile(robot, "test_")
