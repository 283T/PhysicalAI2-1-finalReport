import copy
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from wire_support_vision import StableWireSupports, propose_wire_supports


class WireSupportVisionTest(unittest.TestCase):
    def scene(self, angle=0):
        v, u = np.indices((400, 500))
        x, y = (u - 250) * 0.0005, (v - 200) * 0.0005
        tangent = np.array([np.cos(angle), np.sin(angle), 0])
        along = x * tangent[0] + y * tangent[1]
        across = -x * tangent[1] + y * tangent[0]
        mask = (abs(along) < 0.045) & (abs(across) <= 0.004)
        image = np.full((400, 500, 3), 110, np.uint8)
        image[mask] = [10, 170, 210]
        depth = np.full((400, 500), 0.7, np.float32)
        depth[mask] = 0.5
        return (
            image,
            depth,
            [1000, 0, 250, 0, 1000, 200, 0, 0, 1],
            {"position_m": [0, 0, 0], "optical_to_world_rotation": np.eye(3).tolist()},
            [
                {
                    "plug_id": "P3",
                    "pose_6d": {"center_xyz_m": (tangent * 0.11 + [0, 0, 0.504]).tolist()},
                }
            ],
        )

    def test_support_uses_observed_wire_axis_and_camera_facing_normal(self):
        for angle in (0, 0.6, 1.5):
            with self.subTest(angle=angle):
                rows = propose_wire_supports(*self.scene(angle))
                self.assertEqual(len(rows), 1)
                row = rows[0]
                self.assertAlmostEqual(row["wire_center_xyz_m"][2], 0.504, places=5)
                self.assertAlmostEqual(row["grasp_tcp_pose"][2][3], 0.506, places=5)
                self.assertLess(abs(row["distance_from_connector_m"] - 0.11), 0.001)
                self.assertGreater(
                    np.dot(row["tangent_world"], [np.cos(angle), np.sin(angle), 0]), 0.999
                )
                self.assertTrue(np.allclose(row["surface_normal_world"], [0, 0, -1]))
                rotation = np.asarray(row["grasp_tcp_pose"])[:3, :3]
                self.assertTrue(np.allclose(rotation.T @ rotation, np.eye(3)))
                self.assertAlmostEqual(np.linalg.det(rotation), 1)

    def test_missing_depth_wrong_colour_and_missing_connector_pose_produce_no_target(self):
        for mode in ("depth", "colour", "pose"):
            scene = list(self.scene())
            if mode == "depth":
                scene[1][:] = np.nan
            elif mode == "colour":
                scene[0][:] = 110
            else:
                scene[4][0].pop("pose_6d")
            self.assertEqual(propose_wire_supports(*scene), [])

    def test_selected_distance_changes_observation_region_without_moving_the_wire(self):
        scene = list(self.scene())
        scene[4][0]["pose_6d"]["center_xyz_m"] = [0.17, 0, 0.504]
        rows = propose_wire_supports(*scene, preferred_distance_m=0.17)
        self.assertEqual(len(rows), 1)
        self.assertLess(abs(rows[0]["distance_from_connector_m"] - 0.17), 0.001)
        self.assertAlmostEqual(rows[0]["wire_center_xyz_m"][2], 0.504, places=5)
        for distance in (0.01, 0.21, float("nan")):
            with self.assertRaises(ValueError):
                propose_wire_supports(*scene, preferred_distance_m=distance)

    def test_depth_edge_outliers_are_excluded_without_relaxing_plane_residual(self):
        scene = list(self.scene())
        mask = scene[1] < 0.6
        columns = np.indices(mask.shape)[1]
        scene[1][mask & (columns % 4 == 0)] += 0.008
        rows = propose_wire_supports(*scene)
        self.assertEqual(len(rows), 1)
        self.assertLess(rows[0]["planar_rms_m"], 0.0006)
        self.assertLess(rows[0]["points"], rows[0]["source_points"])
        self.assertAlmostEqual(rows[0]["wire_center_xyz_m"][2], 0.504, places=5)

    def test_housing_width_blob_and_wrong_calibration_are_rejected(self):
        scene = list(self.scene())
        scene[0][180:221, 160:341] = [10, 170, 210]
        scene[1][180:221, 160:341] = 0.5
        self.assertEqual(propose_wire_supports(*scene), [])
        scene[3]["optical_to_world_rotation"][0][0] = -1
        with self.assertRaises(ValueError):
            propose_wire_supports(*scene)

    def test_window_stability_rejects_missing_duplicate_nonfinite_and_slow_drift(self):
        base = propose_wire_supports(*self.scene())
        stability = StableWireSupports(["P3"])
        self.assertFalse(stability.update(base))
        self.assertFalse(stability.update(base))
        self.assertTrue(stability.update(base))
        self.assertFalse(stability.update([]))
        self.assertFalse(stability.update(base * 2))
        invalid = copy.deepcopy(base)
        invalid[0]["grasp_tcp_pose"][0][3] = float("nan")
        self.assertFalse(stability.update(invalid))
        # Each 1.2 mm step is small, but the total three-frame drift exceeds 2 mm.
        for offset in (0, 0.0012, 0.0024):
            moving = copy.deepcopy(base)
            moving[0]["grasp_tcp_pose"][0][3] += offset
            self.assertFalse(stability.update(moving))


if __name__ == "__main__":
    unittest.main()
