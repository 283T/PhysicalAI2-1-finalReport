import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_kinematics import rotation_error, transform
from connector_pose import FACE_CODES
from connector_vision import detect_connectors


class ConnectorPoseTest(unittest.TestCase):
    def render_face(self, face, roll, with_code=True):
        height = width = 256
        k = [700.0, 0, 128.0, 0, 700.0, 128.0, 0, 0, 1.0]
        calibration = {
            "position_m": [0, 0, 1.0],
            "optical_to_world_rotation": [[1, 0, 0], [0, -1, 0], [0, 0, -1]],
        }
        center = np.array([0.0, 0.0, 0.3])
        rotation = transform(rpy=[roll + 0.12, 0.05, 0.37])[:3, :3]
        normal = rotation @ np.array(FACE_CODES[face][1])
        v, u = np.indices((height, width))
        directions = np.stack(((u - 128) / 700, -(v - 128) / 700, -np.ones_like(u)), axis=-1)
        distance = np.dot(normal, center + 0.01 * normal - np.array(calibration["position_m"])) / (
            directions @ normal
        )
        points = np.array(calibration["position_m"]) + distance[:, :, None] * directions
        local = (points - center) @ rotation
        tangent_index = 1 if face.startswith("z_") else 2
        inside = (np.abs(local[:, :, 0]) <= 0.020) & (np.abs(local[:, :, tangent_index]) <= 0.010)
        image = np.full((height, width, 3), 120, dtype=np.uint8)
        image[inside] = [20, 50, 230]
        strip = inside & (np.abs(local[:, :, tangent_index]) < 0.003)
        white = strip & (np.abs(local[:, :, 0] - 0.010) < 0.006)
        code = strip & (np.abs(local[:, :, 0] + 0.010) < 0.006)
        image[white] = 255
        if with_code:
            rgb = np.array([float(value) for value in FACE_CODES[face][0].split()[:3]])
            image[code] = np.round(rgb[::-1] * 255).astype(np.uint8)
        depth = np.full((height, width), 1.0, dtype=np.float32)
        depth[inside] = distance[inside]
        return image, depth, k, calibration, center, rotation

    def test_all_four_faces_recover_center_roll_pitch_yaw_and_grasp_basis(self):
        for face, roll in [
            ("z_positive", 0),
            ("z_negative", math.pi),
            ("y_positive", math.pi / 2),
            ("y_negative", -math.pi / 2),
        ]:
            with self.subTest(face=face):
                image, depth, k, calibration, center, rotation = self.render_face(face, roll)
                rows, _ = detect_connectors(image, depth, k, calibration)
                self.assertEqual(len(rows), 1)
                pose = rows[0]["pose_6d"]
                self.assertEqual(pose["visible_face"], face)
                self.assertLess(np.linalg.norm(np.array(pose["center_xyz_m"]) - center), 0.0015)
                recovered = np.asarray(pose["rotation_world_from_connector"])
                self.assertLess(np.linalg.norm(rotation_error(recovered, rotation)), 0.05)
                self.assertAlmostEqual(np.linalg.det(recovered), 1)
                grasp = np.asarray(pose["grasp_tcp_pose"])
                self.assertAlmostEqual(np.linalg.det(grasp[:3, :3]), 1)
                np.testing.assert_allclose(grasp[:3, 2], -np.asarray(pose["surface_normal_world"]))
                self.assertAlmostEqual(float(grasp[:3, 0] @ recovered[:, 0]), 0)

    def test_missing_face_code_never_claims_full_pose(self):
        image, depth, k, calibration, _, _ = self.render_face("z_positive", 0, with_code=False)
        rows, _ = detect_connectors(image, depth, k, calibration)
        self.assertEqual(len(rows), 1)
        self.assertNotIn("pose_6d", rows[0])
