import math
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_kinematics import transform
from cooperative_transfer import socket_poses
from regrasp_fixture import add_nest_to_world, add_regrasp_nest
from regrasp_plan import top_grasp_relation


class RegraspTests(unittest.TestCase):
    def test_top_grip_closes_across_short_side_in_both_symmetric_orientations(self):
        for flipped in (False, True):
            relation = top_grasp_relation(flipped)
            np.testing.assert_allclose(relation[:3, :3].T @ relation[:3, :3], np.eye(3))
            self.assertAlmostEqual(np.linalg.det(relation[:3, :3]), 1)
            np.testing.assert_allclose(relation[:3, 2], [0, 0, -1])
            self.assertAlmostEqual(abs(relation[1, 0]), 1)
            self.assertAlmostEqual(relation[0, 0], 0)
            np.testing.assert_allclose(relation[:3, 3], [0, 0, -0.006])

    def test_top_grasp_keeps_pins_vertical_for_rotated_socket(self):
        for yaw in (0, 90):
            relation = top_grasp_relation()
            poses = socket_poses(
                {"origin_m": [0.16, 0.08, 0.14]}, {"x_mm": 54, "y_mm": 43, "yaw_deg": yaw}, relation
            )
            connector = poses["seated"] @ np.linalg.inv(relation)
            np.testing.assert_allclose(
                connector, transform([0.214, 0.123, 0.15], [0, 0, math.radians(yaw)]), atol=1e-10
            )
            self.assertAlmostEqual(poses["seated"][2, 3], 0.144)

    def test_nest_supports_ends_and_leaves_central_contact_band_open(self):
        fixture = ET.Element("link")
        info = add_regrasp_nest(fixture)
        self.assertAlmostEqual(info["guide_top_m"], 0.185)
        for collision in fixture.findall("collision"):
            pose = [float(v) for v in collision.findtext("pose").split()]
            size = [float(v) for v in collision.findtext("geometry/box/size").split()]
            if "end_support" in collision.get("name"):
                self.assertGreater(abs(pose[0]) - size[0] / 2, 0.010)
                self.assertAlmostEqual(pose[2] + size[2] / 2, 0.22)
            elif "pin_" in collision.get("name"):
                self.assertLess(pose[2] + size[2] / 2, 0.196)
        negative = fixture.find("collision[@name='regrasp_pin_1_x_-1_collision']")
        positive = fixture.find("collision[@name='regrasp_pin_1_x_1_collision']")
        inner_left = float(negative.findtext("pose").split()[0]) + 0.001
        inner_right = float(positive.findtext("pose").split()[0]) - 0.001
        self.assertAlmostEqual(inner_right - inner_left, 0.008)

    def test_nest_location_rotation_and_duplicate_rejected(self):
        world = ET.fromstring('<world><model name="fixtures"><link name="base"/></model></world>')
        info = add_nest_to_world(world, [0.1, 0.2], yaw=math.pi / 2)
        np.testing.assert_allclose(info["housing_center_m"], [0.1, 0.2, 0.23])
        pose = [
            float(v)
            for v in world.findtext(
                "model/link/collision[@name='regrasp_end_support_1_collision']/pose"
            ).split()
        ]
        np.testing.assert_allclose(pose[:2], [0.1, 0.219], atol=1e-10)
        with self.assertRaises(ValueError):
            add_nest_to_world(world)

    def test_corner_stops_leave_cable_outlet_open(self):
        fixture = ET.Element("link")
        info = add_regrasp_nest(fixture)
        self.assertAlmostEqual(info["cable_outlet_width_m"], 0.010)
        for collision in fixture.findall("collision"):
            if "end_stop" in collision.get("name"):
                pose = [float(v) for v in collision.findtext("pose").split()]
                size = [float(v) for v in collision.findtext("geometry/box/size").split()]
                self.assertGreater(abs(pose[1]) - size[1] / 2, 0.004)
            if "corner" in collision.get("name"):
                pose = [float(v) for v in collision.findtext("pose").split()]
                size = [float(v) for v in collision.findtext("geometry/box/size").split()]
                self.assertGreater(abs(pose[0]) - size[0] / 2, 0.010)

    def test_invalid_nest_clearance_rejected(self):
        for width in (0.004, 0.009, float("nan")):
            with self.subTest(width=width), self.assertRaises(ValueError):
                add_regrasp_nest(ET.Element("link"), channel_width=width)

    def test_pin_relief_leaves_center_passage_and_housing_corner_supports(self):
        fixture = ET.Element("link")
        info = add_regrasp_nest(fixture, pin_relief=True)
        self.assertTrue(info["pin_relief"])
        supports = [c for c in fixture.findall("collision") if "end_support" in c.get("name")]
        self.assertEqual(len(supports), 4)
        for support in supports:
            xyz = [float(v) for v in support.findtext("pose").split()[:3]]
            size = [float(v) for v in support.findtext("geometry/box/size").split()]
            self.assertAlmostEqual(abs(xyz[1]) - size[1] / 2, 0.005)
            self.assertAlmostEqual(xyz[2] + size[2] / 2, 0.22)
            self.assertLess(abs(xyz[1]) - size[1] / 2, 0.010)


if __name__ == "__main__":
    unittest.main()
