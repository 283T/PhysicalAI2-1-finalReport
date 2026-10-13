"""Bounded image-derived correction with continuous seeded inverse kinematics."""

import numpy as np
from arm_kinematics import rotation_error
from cooperative_transfer import pose_samples, rigid_pose


def desired_tool_pose(detection, connector_from_tool):
    body = np.eye(4)
    body[:3, :3] = detection["rotation_world_from_connector"]
    body[:3, 3] = detection["center_xyz_m"]
    return rigid_pose(body) @ rigid_pose(connector_from_tool)


def correction_path(arm, current, goal):
    """Limit one correction to 5 mm / 0.10 rad, without changing IK branches."""
    current = np.asarray(current, dtype=float)
    if current.ndim != 1 or not len(current) or not np.isfinite(current).all():
        raise ValueError("Finite measured joints required")
    start, goal = rigid_pose(arm.forward(current)), rigid_pose(goal)
    distance = float(np.linalg.norm(goal[:3, 3] - start[:3, 3]))
    angle = float(np.linalg.norm(rotation_error(goal[:3, :3], start[:3, :3])))
    if distance > 0.005 or angle > 0.10:
        raise ValueError("Image-derived correction exceeds 5 mm / 0.10 rad limits")
    path = [current.copy()]
    for pose in pose_samples(start, goal, spacing=0.001, angular_spacing=0.01)[1:]:
        result = arm.inverse(pose, seed=path[-1], attempts=1)
        joints = np.asarray(result["q_rad"], dtype=float)
        if (
            not result["success"]
            or joints.shape != current.shape
            or not np.isfinite(joints).all()
            or np.max(np.abs(joints - path[-1])) > 0.15
        ):
            raise ValueError("Unreachable or discontinuous pre-close correction")
        path.append(joints)
    return np.asarray(path)
