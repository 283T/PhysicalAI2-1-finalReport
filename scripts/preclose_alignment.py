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
        result = arm.inverse(
            pose, seed=path[-1], attempts=1, position_tolerance=0.0001, angle_tolerance=0.002
        )
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


def correction_budget(used, starts, goals):
    """Bound the sum of image-commanded Cartesian corrections, across all retries."""
    if set(starts) != set(goals) or not starts or not set(used) <= set(starts):
        raise ValueError("Matching correction arms required")
    totals = {}
    for name, start in starts.items():
        start, goal = rigid_pose(start), rigid_pose(goals[name])
        previous = used.get(name, {"translation_m": 0.0, "rotation_rad": 0.0})
        values = np.asarray([previous["translation_m"], previous["rotation_rad"]], dtype=float)
        if not np.isfinite(values).all() or min(values) < 0:
            raise ValueError("Invalid accumulated correction budget")
        distance = values[0] + np.linalg.norm(goal[:3, 3] - start[:3, 3])
        angle = values[1] + np.linalg.norm(rotation_error(goal[:3, :3], start[:3, :3]))
        if distance > 0.005 or angle > 0.10:
            raise ValueError("Cumulative image corrections exceed 5 mm / 0.10 rad budget")
        totals[name] = {"translation_m": float(distance), "rotation_rad": float(angle)}
    return totals


def correction_tracking(position_error, angle_error, joint_error, fk_error):
    """Separate preferred precision from existing move limits; neither accepts a grasp."""
    values = np.asarray([position_error, angle_error, joint_error, fk_error], dtype=float)
    finite = bool(np.isfinite(values).all() and np.all(values >= 0))
    within_move_limits = finite and bool(np.all(values < [0.002, 0.025, 0.01, 0.001]))
    return {
        "within_move_limits": within_move_limits,
        "precision_reached": bool(
            within_move_limits and position_error < 0.0005 and angle_error < 0.005
        ),
        "position_error_m": float(position_error),
        "angle_error_rad": float(angle_error),
        "joint_error_rad": float(joint_error),
        "fk_error_m": float(fk_error),
    }
