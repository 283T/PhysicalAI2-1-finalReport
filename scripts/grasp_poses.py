"""Candidate grip/hover transforms preserving the measured connector-to-tool relation."""

import math

import numpy as np
from arm_kinematics import transform


def candidate_poses(observation, socket_center, socket_yaw):
    pose = observation.get("pose_6d")
    if pose is None:
        raise ValueError("A coded-face 6D connector pose is required for a grip candidate")
    connector = np.eye(4)
    connector[:3, :3] = pose["rotation_world_from_connector"]
    connector[:3, 3] = pose["center_xyz_m"]
    tool = np.asarray(pose["grasp_tcp_pose"], dtype=float)
    normal = np.asarray(pose["surface_normal_world"], dtype=float)
    for matrix in (connector, tool):
        if (
            matrix.shape != (4, 4)
            or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1])
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1)
        ):
            raise ValueError("Connector/grasp must be rigid transforms")
    if (
        normal.shape != (3,)
        or not np.isfinite(normal).all()
        or not np.isclose(np.linalg.norm(normal), 1)
    ):
        raise ValueError("Visible normal must be a unit vector")
    if not np.allclose(tool[:3, 2], -normal, atol=1e-6):
        raise ValueError("Grip approach must point towards the visible face")
    relation = np.linalg.inv(connector) @ tool
    hover = tool.copy()
    hover[:3, 3] += 0.10 * normal
    socket_connector = transform(socket_center, [0, 0, math.radians(socket_yaw)])
    socket_tool = socket_connector @ relation
    return {
        "grasp_tcp_pose": tool.tolist(),
        "pickup_hover_pose": hover.tolist(),
        "socket_hover_pose": socket_tool.tolist(),
        "connector_from_tool": relation.tolist(),
        "scope": "Prototype geometric candidate; no force/contact, IK path or grasp confirmation",
    }
