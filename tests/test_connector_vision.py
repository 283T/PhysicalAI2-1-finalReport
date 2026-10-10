import json
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from connector_vision import detect_connectors


class ConnectorVisionTest(unittest.TestCase):
    def setUp(self):
        self.calibration = json.loads((ROOT/'configs/infeed_camera.json').read_text())
        self.k = [400., 0., 160., 0., 400., 120., 0., 0., 1.]
        self.depth = np.full((240, 320), .7, dtype=np.float32)

    def image(self, marker=True, duplicate=False):
        image = np.full((240, 320, 3), 120, dtype=np.uint8)
        colors = [(20, 50, 230), (230, 50, 20), (20, 200, 230)]
        for i, color in enumerate(colors):
            x, y = 70+i*80, 80
            cv2.rectangle(image, (x-18, y-8), (x+18, y+8), color, -1)
            cv2.line(image, (x-55, y), (x-18, y), color, 4)
            if marker:
                cv2.rectangle(image, (x+6, y-3), (x+13, y+3), (255, 255, 255), -1)
        if duplicate:
            cv2.rectangle(image, (55, 150), (91, 166), colors[0], -1)
            cv2.rectangle(image, (79, 155), (86, 161), (255, 255, 255), -1)
        return image

    def test_all_ids_surface_height_and_directed_yaw(self):
        detections, _ = detect_connectors(self.image(), self.depth, self.k, self.calibration)
        self.assertEqual({row['plug_id'] for row in detections}, {'P1', 'P2', 'P3'})
        for row in detections:
            self.assertAlmostEqual(row['surface_xyz_m'][2], .2, places=5)
            self.assertAlmostEqual(row['yaw_rad'], -np.pi/2, places=3)

    def test_missing_marker_cannot_claim_orientation(self):
        detections, _ = detect_connectors(self.image(marker=False), self.depth, self.k, self.calibration)
        self.assertEqual(detections, [])

    def test_invalid_depth_and_duplicate_color_are_rejected(self):
        missing_depth = self.depth.copy()
        missing_depth[:] = np.nan
        detections, _ = detect_connectors(self.image(), missing_depth, self.k, self.calibration)
        self.assertEqual(detections, [])
        detections, _ = detect_connectors(self.image(duplicate=True), self.depth, self.k, self.calibration)
        self.assertEqual({row['plug_id'] for row in detections}, {'P2', 'P3'})

    def test_unregistered_depth_rejected(self):
        with self.assertRaises(ValueError):
            detect_connectors(self.image(), self.depth[:10], self.k, self.calibration)

    def test_invalid_camera_intrinsics_rejected(self):
        invalid = self.k.copy()
        invalid[0] = 0
        with self.assertRaises(ValueError):
            detect_connectors(self.image(), self.depth, invalid, self.calibration)


if __name__ == '__main__':
    unittest.main()
