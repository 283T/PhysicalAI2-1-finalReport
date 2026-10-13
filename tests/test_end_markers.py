import math
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_kinematics import rotation_error, transform
from end_markers import (
    TAG_SIZE_M,
    add_end_markers,
    connector_from_marker,
    detect_end_markers,
    marker_image,
)


class EndMarkerTests(unittest.TestCase):
    def render(self, marker_id, x=0):
        size = 400
        k = [800.0, 0, 200.0, 0, 800.0, 200.0, 0, 0, 1.0]
        calibration = {"position_m": [0, 0, 0], "optical_to_world_rotation": np.eye(3).tolist()}
        marker = transform([x, 0, 0.35], [math.pi + 0.12, 0.1, 0.24])
        half = TAG_SIZE_M / 2
        template = np.array(
            [[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]]
        )
        points = template @ marker[:3, :3].T + marker[:3, 3]
        corners = points[:, :2] / points[:, 2, None] * 800 + 200
        source = np.array([[0, 0], [119, 0], [119, 119], [0, 119]], np.float32)
        mapping = cv2.getPerspectiveTransform(source, corners.astype(np.float32))
        image = cv2.warpPerspective(
            marker_image(marker_id), mapping, (size, size), flags=cv2.INTER_NEAREST, borderValue=255
        )
        v, u = np.indices((size, size))
        rays = np.stack(((u - 200) / 800, (v - 200) / 800, np.ones_like(u)), axis=-1)
        normal = marker[:3, 2]
        depth = (marker[:3, 3] @ normal) / (rays @ normal)
        truth = marker @ np.linalg.inv(connector_from_marker(marker_id))
        return (
            cv2.cvtColor(image, cv2.COLOR_GRAY2BGR),
            depth.astype(np.float32),
            k,
            calibration,
            truth,
        )

    def test_six_ids_recover_both_ends_and_connector_identity(self):
        for marker_id in range(6):
            with self.subTest(marker_id=marker_id):
                image, depth, k, calibration, truth = self.render(marker_id)
                rows, _ = detect_end_markers(image, depth, k, calibration)
                self.assertEqual(len(rows), 1)
                self.assertEqual(rows[0]["plug_id"], "P" + str(marker_id // 2 + 1))
                pose = rows[0]["pose_6d"]
                self.assertLess(
                    np.linalg.norm(np.asarray(pose["center_xyz_m"]) - truth[:3, 3]), 0.0015
                )
                self.assertLess(
                    np.linalg.norm(
                        rotation_error(
                            np.asarray(pose["rotation_world_from_connector"]), truth[:3, :3]
                        )
                    ),
                    0.05,
                )
                self.assertNotIn("grasp_tcp_pose", pose)

    def test_missing_depth_or_inconsistent_metric_tag_size_is_rejected(self):
        image, depth, k, calibration, _ = self.render(0)
        for invalid in (np.full_like(depth, np.nan), depth * 2):
            self.assertEqual(detect_end_markers(image, invalid, k, calibration)[0], [])

    def test_duplicate_identity_is_not_silently_selected(self):
        first, depth, k, calibration, _ = self.render(0, x=-0.04)
        second, _, _, _, _ = self.render(0, x=0.04)
        self.assertEqual(
            detect_end_markers(np.minimum(first, second), depth, k, calibration)[0], []
        )

    def test_end_tags_add_visuals_without_changing_collisions(self):
        terminal = ET.fromstring('<link><collision name="housing"/></link>')
        before = ET.tostring(terminal.find("collision"))
        add_end_markers(terminal, "P1")
        self.assertGreater(len(terminal.findall("visual")), 30)
        self.assertEqual(len(terminal.findall("collision")), 1)
        self.assertEqual(ET.tostring(terminal.find("collision")), before)


if __name__ == "__main__":
    unittest.main()
