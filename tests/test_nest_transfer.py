import math
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_kinematics import transform
from plan_nest_transfer import nest_connector_pose
from regrasp_fixture import add_regrasp_nest


class NestTransferTests(unittest.TestCase):
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
