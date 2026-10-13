import math
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from gripper_model import OPEN_TRAVEL_M, add_controllers, add_gripper


class GripperControllerTests(unittest.TestCase):
    def test_single_front_fence_keeps_both_rear_fences_and_updates_mass(self):
        for side in ("left", "right"):
            robot = ET.fromstring('<robot><link name="root"/></robot>')
            add_gripper(robot, "test_", "root", capture_fences=True, front_fence_side=side)
            for name in ("left", "right"):
                finger = robot.find(f"link[@name='test_finger_{name}']")
                self.assertIsNotNone(finger.find("collision[@name='rear_fence']"))
                self.assertEqual(
                    finger.find("collision[@name='front_fence']") is not None, name == side
                )
                self.assertAlmostEqual(
                    float(finger.find("inertial/mass").get("value")),
                    0.012 if name == side else 0.011,
                )

    def test_invalid_or_inactive_single_fence_configuration_is_rejected(self):
        for options in ({"front_fence_side": "top"}, {"front_fence_side": "right"}):
            with self.assertRaises(ValueError):
                add_gripper(ET.Element("robot"), "test_", "root", **options)

    def test_capture_slot_matches_housing_and_retracts_with_the_jaws(self):
        robot = ET.fromstring('<robot><link name="root"/></robot>')
        add_gripper(robot, "test_", "root", split_contacts=True, capture_fences=True)
        for sign, name in ((-1, "left"), (1, "right")):
            faces = []
            for kind in ("rear_fence", "front_fence"):
                fence = robot.find(f"link[@name='test_finger_{name}']/collision[@name='{kind}']")
                xyz = [float(v) for v in fence.find("origin").get("xyz").split()]
                size = [float(v) for v in fence.find("geometry/box").get("size").split()]
                faces.append(
                    xyz[2] - 0.08 + (size[2] / 2 if kind == "rear_fence" else -size[2] / 2)
                )
                self.assertGreater(0.012 + OPEN_TRAVEL_M + sign * xyz[0] - size[0] / 2, 0.010)
            self.assertAlmostEqual(faces[1] - faces[0], 0.021)
            self.assertAlmostEqual(sum(faces) / 2, 0.002)
            self.assertLess(faces[0], -0.002)
            self.assertGreater(faces[1], 0.006)

    def test_backstop_opens_clear_of_housing_and_avoids_pin_plane(self):
        robot = ET.fromstring('<robot><link name="root"/></robot>')
        add_gripper(robot, "test_", "root", split_contacts=True, backstop=True)
        for sign, name in ((-1, "left"), (1, "right")):
            stop = robot.find(f"link[@name='test_finger_{name}']/collision[@name='backstop']")
            xyz = [float(v) for v in stop.find("origin").get("xyz").split()]
            size = [float(v) for v in stop.find("geometry/box").get("size").split()]
            tip_relative_face = xyz[2] + size[2] / 2 - 0.08
            self.assertAlmostEqual(-0.008 - tip_relative_face, 0.0003)
            self.assertLess(tip_relative_face, 0)  # Pins lie at tool-Z=+2 mm.
            open_inner = 0.012 + OPEN_TRAVEL_M + sign * xyz[0] - size[0] / 2
            self.assertGreater(open_inner, 0.010)
            self.assertLess(size[1] / 2 + 0.002, 0.016)  # Nest corner inner X.

    def test_gain_comparison_preserves_bounded_force_and_other_settings(self):
        models = []
        for gain in (1000, 3000):
            model = ET.Element("model")
            add_controllers(model, "test_", "/test", p_gain=gain)
            for plugin in model.findall("plugin"):
                self.assertEqual(float(plugin.findtext("cmd_max")), 12)
                self.assertEqual(float(plugin.findtext("cmd_min")), -12)
                self.assertEqual(float(plugin.findtext("p_gain")), gain)
                plugin.find("p_gain").text = "same"
            models.append(ET.tostring(model))
        self.assertEqual(models[0], models[1])

    def test_nonfinite_or_nonpositive_gains_are_rejected(self):
        for gain in (0, -1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                add_controllers(ET.Element("model"), "test_", "/test", p_gain=gain)


if __name__ == "__main__":
    unittest.main()
