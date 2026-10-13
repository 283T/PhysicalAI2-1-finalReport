import copy
import json
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from inspection_camera import add_inspection_camera, validate_calibration


class InspectionCameraTests(unittest.TestCase):
    def calibration(self):
        return json.loads((ROOT / "configs/inspection_camera.json").read_text())

    def test_both_configs_have_matching_native_and_optical_axes(self):
        for name in ("infeed_camera", "inspection_camera"):
            validate_calibration(json.loads((ROOT / f"configs/{name}.json").read_text()))
        data = self.calibration()
        forward = np.asarray(data["optical_to_world_rotation"])[:, 2]
        direction = np.array([-0.30, 0.05, 0.20]) - data["position_m"]
        np.testing.assert_allclose(forward, direction / np.linalg.norm(direction), atol=1e-10)

    def test_inconsistent_extrinsics_and_invalid_fields_are_rejected(self):
        for key, value in (
            ("optical_to_world_rotation", np.eye(3).tolist()),
            ("position_m", [0, 0, float("nan")]),
            ("frame", "camera"),
            ("width", -1),
            ("topic", "relative"),
        ):
            data = self.calibration()
            data[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_calibration(data)

    def test_second_camera_does_not_modify_the_pickup_camera(self):
        world = ET.fromstring(
            '<world><model name="overhead_camera"><static>true</static>'
            '<pose>1 2 3 0 0 0</pose><link name="camera_link"><pose>0 0 1 0 0 0</pose>'
            '<sensor name="rgb" type="rgbd_camera"><pose>1 0 0 0 0 0</pose>'
            "<topic>/factory/rgbd</topic><update_rate>5</update_rate><camera>"
            "<horizontal_fov>1.1</horizontal_fov><image><width>640</width><height>480</height>"
            "</image></camera></sensor></link></model></world>"
        )
        original = ET.tostring(copy.deepcopy(world.find("model")))
        add_inspection_camera(world, self.calibration())
        self.assertEqual(ET.tostring(world.find("model")), original)
        camera = world.find("model[@name='inspection_camera']")
        self.assertIsNone(camera.find("pose"))
        sensor = camera.find("link/sensor")
        self.assertIsNone(sensor.find("pose"))
        self.assertEqual(sensor.findtext("topic"), "/factory/inspection_rgbd")
        self.assertEqual(sensor.findtext("update_rate"), "1")
        with self.assertRaises(ValueError):
            add_inspection_camera(world, self.calibration())


if __name__ == "__main__":
    unittest.main()
