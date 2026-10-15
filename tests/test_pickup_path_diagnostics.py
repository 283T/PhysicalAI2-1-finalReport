import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from cooperative_pick import PickupPathError, prepare_pickup_paths
from plan_wire_pickup import plan_wire_pickup, upright_wire_target


class PointArm:
    base = np.eye(4)

    def forward(self, q):
        pose = np.eye(4)
        pose[:3, 3] = q[:3]
        return pose

    def inverse(self, pose, **kwargs):
        return {"success": True, "q_rad": [*pose[:3, 3], 0, 0, 0]}


class PointBounds:
    def boxes(self, q):
        return [("test_link", (np.array(q[:3]), np.eye(3), np.full(3, 0.01)))]


class PickupPathDiagnosticsTest(unittest.TestCase):
    def test_upright_wire_grip_clears_table_and_uses_projected_square_width(self):
        angle = 0.13
        c, s = np.cos(angle), np.sin(angle)
        measured = np.eye(4)
        measured[:3, :3] = [[-c, 0, -s], [0, 1, 0], [s, 0, -c]]
        center = np.array([-0.26, 0.12, 0.1245])
        measured[:3, 3] = center - np.array([s, 0, c]) * 0.002
        target, width = upright_wire_target(
            {"grasp_tcp_pose": measured.tolist(), "wire_center_xyz_m": center.tolist()}
        )
        self.assertAlmostEqual(width, 0.008 * (c + s))
        tilted_tip = measured[:3, 3] + measured[:3, 0] * -0.03
        vertical_tip = target[:3, 3] + target[:3, 0] * -0.03
        self.assertLess(tilted_tip[2], 0.12)
        self.assertGreater(vertical_tip[2], 0.123)
        self.assertTrue(np.allclose(target[:3, 2], [0, 0, -1]))

    def test_execution_planner_rejects_single_frame_candidates_by_default(self):
        with self.assertRaisesRegex(ValueError, "Three stable"):
            plan_wire_pickup(
                {"left": PointArm(), "right": PointArm()},
                {"left": PointBounds(), "right": PointBounds()},
                Path("not_read.sdf"),
                {"frame": "world", "source": "registered_rgbd_color_and_plane", "stable_frames": 0},
                {"left": np.zeros(6), "right": np.zeros(6)},
                "left",
                "P3",
            )

    def plan(self, x):
        grip = np.eye(4)
        grip[0, 3] = x
        hover = grip.copy()
        hover[2, 3] = 0.1
        return prepare_pickup_paths(
            {"left": PointArm()},
            {"left": np.zeros(6)},
            {"left": hover},
            {"left": grip},
            {"left": PointBounds(), "right": PointBounds()},
            parked={"right": np.array([0.5, 0, 0, 0, 0, 0])},
            include_lift=False,
        )

    def test_approach_collision_remains_rejected_when_individual_lift_is_omitted(self):
        with self.assertRaises(PickupPathError) as caught:
            self.plan(0.5)
        candidates = caught.exception.diagnostics["combinations"]
        self.assertGreater(len(candidates), 0)
        self.assertTrue(all(not c["screens"]["approach"]["clear"] for c in candidates))
        self.assertTrue(all(c["screens"]["approach"]["links"] for c in candidates))

    def test_safe_approach_without_standalone_lift_does_not_plan_that_motion(self):
        paths, report = self.plan(0.2)
        self.assertEqual(set(paths), {"hover", "approach"})
        self.assertTrue(all(s["clear"] for s in report["screens"].values()))


if __name__ == "__main__":
    unittest.main()
