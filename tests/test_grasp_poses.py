import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_kinematics import transform
from grasp_poses import candidate_poses


class GraspPoseTest(unittest.TestCase):
    def test_socket_tool_preserves_gripped_connector_relation_after_unrolling(self):
        connector = transform([-0.3, 0.1, 0.13], [-np.pi / 2, 0.05, 0.4])
        normal = -connector[:3, 1]
        x = connector[:3, 0]
        tool = np.eye(4)
        tool[:3, :3] = np.column_stack((np.cross(normal, x), x, -normal))
        tool[:3, 3] = connector[:3, 3] + 0.002 * normal
        observation = {
            "pose_6d": {
                "center_xyz_m": connector[:3, 3].tolist(),
                "rotation_world_from_connector": connector[:3, :3].tolist(),
                "surface_normal_world": normal.tolist(),
                "grasp_tcp_pose": tool.tolist(),
            }
        }
        result = candidate_poses(observation, [0.2, 0.2, 0.3], 90)
        relation = np.asarray(result["connector_from_tool"])
        np.testing.assert_allclose(connector @ relation, tool, atol=1e-10)
        socket = transform([0.2, 0.2, 0.3], [0, 0, np.pi / 2])
        np.testing.assert_allclose(socket @ relation, result["socket_hover_pose"], atol=1e-10)
        hover = np.asarray(result["pickup_hover_pose"])
        np.testing.assert_allclose(hover[:3, 3] - tool[:3, 3], 0.10 * normal, atol=1e-10)

    def test_planar_only_observation_is_not_a_grasp(self):
        with self.assertRaises(ValueError):
            candidate_poses({"surface_xyz_m": [0, 0, 0.1], "yaw_rad": 0}, [0.2, 0.2, 0.3], 0)
