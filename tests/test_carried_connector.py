import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_kinematics import transform
from carried_connector import CarriedConnector
from fixture_collision_bounds import FixtureBounds


class CarriedConnectorTests(unittest.TestCase):
    def world(self, path, fixture_name="socket_J1_0_0_collision", pin_obstacle=False):
        fixture = (
            "<pose>0.01 0 -0.03 0 0 0</pose><geometry><box><size>0.001 0.01 0.001</size></box></geometry>"
            if pin_obstacle
            else "<pose>0 0 -0.03 0 0 0</pose><geometry><box><size>0.008 0.02 0.04</size></box></geometry>"
        )
        path.write_text(
            f'''<sdf><world><model name="fixtures"><static>true</static><link name="fixed"><collision name="{fixture_name}">{fixture}</collision></link></model><model name="wire_harness_multi"><pose>999 999 999 0 0 0</pose><link name="P3"><pose>500 500 500 0 0 0</pose><collision name="housing_collision"><geometry><box><size>0.04 0.02 0.02</size></box></geometry></collision><collision name="pin_0_collision"><pose>0.01 0 -0.03 0 0 0</pose><geometry><cylinder><radius>0.002</radius><length>0.04</length></cylinder></geometry></collision><collision name="pin_1_collision"><pose>-0.01 0 -0.03 0 0 0</pose><geometry><cylinder><radius>0.002</radius><length>0.04</length></cylinder></geometry></collision></link></model></world></sdf>'''
        )

    def test_intrinsic_model_does_not_use_initial_object_placement(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "world.sdf"
            self.world(path)
            body = CarriedConnector(path, "P3")
            self.assertTrue(np.allclose(body.boxes(np.eye(4))[0][2][0], [0, 0, 0]))
            self.assertTrue(np.allclose(body.boxes(transform([1, 2, 3]))[1][2][0], [1.01, 2, 2.97]))

    def test_only_housing_contact_on_selected_rim_near_seated_pose_is_expected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "world.sdf"
            self.world(path)
            body = CarriedConnector(path, "P3")
            fixtures = FixtureBounds(path)
            goal = {"socket_id": "J1", "connector_pose": np.eye(4).tolist()}
            self.assertIsNotNone(body.fixture_overlap(np.eye(4), fixtures)[0])
            blocked, intended = body.fixture_overlap(np.eye(4), fixtures, seating=goal)
            self.assertIsNone(blocked)
            self.assertEqual(len(intended), 1)
            self.assertIsNotNone(
                body.fixture_overlap(transform([0.004, 0, 0]), fixtures, seating=goal)[0]
            )
            self.world(path, fixture_name="socket_J2_0_0_collision")
            self.assertIsNotNone(
                body.fixture_overlap(np.eye(4), FixtureBounds(path), seating=goal)[0]
            )

    def test_target_contact_permission_cannot_hide_a_pin_obstacle(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "world.sdf"
            self.world(path, pin_obstacle=True)
            body = CarriedConnector(path, "P3")
            fixtures = FixtureBounds(path)
            blocked, _ = body.fixture_overlap(
                np.eye(4),
                fixtures,
                seating={"socket_id": "J1", "connector_pose": np.eye(4).tolist()},
            )
            self.assertTrue(blocked["connector_shape"].startswith("pin_"))

            class Arm:
                def forward(self, q):
                    return transform(q[:3])

            result = body.screen(Arm(), [[-0.1, 0, 0], [0.1, 0, 0]], np.eye(4), fixtures)
            self.assertFalse(result["clear"])
            self.assertGreater(result["fraction"], 0)
            self.assertLess(result["fraction"], 1)

    def test_unsupported_collision_shape_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "world.sdf"
            self.world(path)
            tree = ET.parse(path)
            geometry = tree.find("world/model[@name='wire_harness_multi']/link/collision/geometry")
            geometry.remove(geometry.find("box"))
            ET.SubElement(geometry, "mesh")
            tree.write(path)
            with self.assertRaises(ValueError):
                CarriedConnector(path, "P3")
