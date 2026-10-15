import math
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from gripper_model import add_gripper
from plan_wire_pickup import wire_grasp_target
from stepped_wire_profile import stepped_profile
from wire_contact_profile import validate_deep_wire_profile


class WireContactProfileTests(unittest.TestCase):
    def profile(self):
        robot = ET.Element("robot")
        add_gripper(
            robot,
            "test_",
            "root",
            split_contacts=True,
            pad_inward_extension_m=0.005,
            wire_contact_length_m=0.050,
        )
        return stepped_profile(robot, "test_")

    def test_wire_cross_section_must_fit_entirely_below_shoulder(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "finger.urdf"
            ET.ElementTree(self.profile()).write(path)
            report = validate_deep_wire_profile(path, "test_", 0.019)
            self.assertEqual(report["required_local_z_interval_m"], [0.056, 0.066])
            self.assertAlmostEqual(report["rest_gap_m"], 0.006)
            for finger in report["fingers"].values():
                self.assertEqual(len(finger["active_local_y_intervals_m"]), 2)
            for depth in (0.013, 0.017, 0, math.nan, math.inf, 0.025):
                with self.subTest(depth=depth), self.assertRaises(ValueError):
                    validate_deep_wire_profile(path, "test_", depth)

    def test_depth_beyond_old_limit_requires_real_matching_geometry(self):
        proposal = {
            "grasp_tcp_pose": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
            "wire_center_xyz_m": [0.1, 0.2, 0.3],
        }
        with self.assertRaises(ValueError):
            wire_grasp_target(proposal, tip_depth_m=0.019)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "finger.urdf"
            ET.ElementTree(self.profile()).write(path)
            target, width = wire_grasp_target(
                proposal, tip_depth_m=0.019, wire_profile=path, profile_prefix="test_"
            )
            self.assertAlmostEqual(target[2, 3], 0.319)
            self.assertAlmostEqual(width, 0.008)
            with self.assertRaises(ValueError):
                wire_grasp_target(
                    proposal,
                    upright=True,
                    tip_depth_m=0.019,
                    wire_profile=path,
                    profile_prefix="test_",
                )
            with self.assertRaises(ValueError):
                wire_grasp_target(
                    proposal, tip_depth_m=0.019, wire_profile=path, profile_prefix="different_"
                )

    def test_fences_or_nonbare_profiles_are_not_a_depth_permission(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "finger.urdf"
            robot = self.profile()
            robot.find("link[@name='test_finger_left']/collision").set("name", "backstop")
            ET.ElementTree(robot).write(path)
            with self.assertRaises(ValueError):
                validate_deep_wire_profile(path, "test_", 0.019)
