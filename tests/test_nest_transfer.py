import math
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_kinematics import transform
from plan_nest_transfer import nest_connector_pose, plan_to_nest
from regrasp_fixture import add_regrasp_nest
from test_cooperative_transfer import PoseArm


class NestTransferTests(unittest.TestCase):
    def test_intermediate_rotation_avoids_rejected_endpoint_rotation_with_rigid_frames(self):
        class RotationRegionArm(PoseArm):
            def inverse(self, target, seed, attempts=1, **kwargs):
                result = super().inverse(target, seed, attempts=attempts, **kwargs)
                if abs(result["q_rad"][3] - seed[3]) > 1e-6 and not 0.15 <= target[0, 3] <= 0.25:
                    result["success"] = False
                return result

        class PointBounds:
            def boxes(self, q):
                return [("tool", (np.asarray(q[:3]), np.eye(3), np.full(3, 0.001)))]

        with tempfile.TemporaryDirectory() as directory:
            world = self.write_world(directory, center=(0.4, 0))
            plan = plan_to_nest(
                {"a": RotationRegionArm(), "b": PoseArm()},
                {"a": PointBounds(), "b": PointBounds()},
                world,
                {"a": [0, 0, 0.4, 0], "b": [0, 0.2, 0.4, 0]},
                "a",
                np.eye(4),
                support_orbits=(None,),
                intermediate_rotations=(0.5,),
            )
            self.assertTrue(plan["success"])
            chosen = plan["attempts"][plan["selected_attempt"]]
            self.assertEqual(chosen["order"], "intermediate_rotation")
            self.assertEqual(len(chosen["phases"]), 6)
            self.assertTrue(all(p["pair"]["success"] for p in chosen["phases"]))
            self.assertTrue(
                all(p["pair"]["constraint"] == "relative_frame" for p in chosen["phases"])
            )

    def write_world(self, folder, center=(0.08, -0.04), yaw=0, split=False):
        root = ET.fromstring(
            '<sdf><world><model name="fixtures"><static>true</static><link name="base"/></model></world></sdf>'
        )
        add_regrasp_nest(root.find("world/model/link"), center=center, yaw=yaw, pin_relief=split)
        path = Path(folder) / "world.sdf"
        ET.ElementTree(root).write(path)
        return path

    def test_seat_is_derived_from_actual_rotated_support_faces(self):
        with tempfile.TemporaryDirectory() as directory:
            for yaw in (0, math.pi / 2):
                for split in (False, True):
                    path = self.write_world(directory, center=(-0.1, 0.2), yaw=yaw, split=split)
                    np.testing.assert_allclose(
                        nest_connector_pose(path),
                        transform([-0.1, 0.2, 0.23], [0, 0, yaw]),
                        atol=1e-10,
                    )

    def test_single_transfer_screens_parked_arm_and_keeps_its_joints_constant(self):
        class PointBounds:
            def boxes(self, q):
                return [("tool", (np.asarray(q[:3]), np.eye(3), np.full(3, 0.001)))]

        with tempfile.TemporaryDirectory() as directory:
            world = self.write_world(directory)
            args = (
                {"active": PoseArm()},
                {"active": PointBounds(), "idle": PointBounds()},
                world,
                {"active": [0.1, -0.3, 0.4, 0]},
                "active",
                np.eye(4),
            )
            plan = plan_to_nest(*args, parked={"idle": [2, 2, 2, 0]})
            self.assertTrue(plan["success"])
            self.assertEqual(plan["mode"], "single_with_parked_arm")
            np.testing.assert_allclose(plan["goal_connector_pose"], transform([0.08, -0.04, 0.23]))
            blocked = plan_to_nest(*args, parked={"idle": [0.08, -0.04, 0.23, 0]})
            self.assertFalse(blocked["success"])
            with self.assertRaises(ValueError):
                plan_to_nest(*args)

    def test_missing_or_uneven_supports_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_world(directory)
            tree = ET.parse(path)
            support = tree.find(
                "world/model/link/collision[@name='regrasp_end_support_1_collision']"
            )
            pose = support.find("pose")
            values = pose.text.split()
            values[2] = str(float(values[2]) + 0.01)
            pose.text = " ".join(values)
            tree.write(path)
            with self.assertRaises(ValueError):
                nest_connector_pose(path)
            tree.find("world/model/link").remove(support)
            tree.write(path)
            with self.assertRaises(ValueError):
                nest_connector_pose(path)


if __name__ == "__main__":
    unittest.main()
