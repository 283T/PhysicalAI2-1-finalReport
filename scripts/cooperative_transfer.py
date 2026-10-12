"""Preflight coordinated transport without stretching the two held endpoint frames."""

import math

import numpy as np
from arm_kinematics import axis_rotation, rotation_error, transform
from cartesian_path import interpolate_joints


def rigid_pose(pose):
    pose = np.asarray(pose, dtype=float)
    if (
        pose.shape != (4, 4)
        or not np.isfinite(pose).all()
        or not np.allclose(pose[3], [0, 0, 0, 1])
        or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-6)
        or not np.isclose(np.linalg.det(pose[:3, :3]), 1)
    ):
        raise ValueError("A finite rigid pose is required")
    return pose


def pose_samples(start, goal, spacing=0.005, angular_spacing=0.025):
    """Straight position and shortest SO(3) rotation, including a half turn."""
    start, goal = rigid_pose(start), rigid_pose(goal)
    if not np.isfinite([spacing, angular_spacing]).all() or min(spacing, angular_spacing) <= 0:
        raise ValueError("Positive finite sampling limits required")
    vector = rotation_error(goal[:3, :3], start[:3, :3])
    angle = np.linalg.norm(vector)
    steps = max(
        1,
        math.ceil(np.linalg.norm(goal[:3, 3] - start[:3, 3]) / spacing),
        math.ceil(angle / angular_spacing),
    )
    return [intermediate_pose(start, goal, fraction) for fraction in np.linspace(0, 1, steps + 1)]


def paired_path(
    arms, current, primary, goal, max_joint_step=0.15, support_orbit=None, support_rise=0.0
):
    """Rigid transport, or a world-Z support orbit with unchanged support orientation."""
    if len(arms) != 2 or set(arms) != set(current) or primary not in arms:
        raise ValueError("Need two current arm states and a primary arm")
    if not math.isfinite(max_joint_step) or max_joint_step <= 0:
        raise ValueError("Positive finite joint step required")
    if not math.isfinite(support_rise) or (support_rise != 0 and support_orbit is None):
        raise ValueError("Finite support rise requires an independent support orbit")
    starts = {name: rigid_pose(arm.forward(current[name])) for name, arm in arms.items()}
    paths = {name: [np.asarray(current[name], dtype=float)] for name in arms}
    reference_inverse = np.linalg.inv(starts[primary])
    poses = pose_samples(starts[primary], goal)
    if support_orbit is not None:
        if not math.isfinite(support_orbit):
            raise ValueError("Support orbit must be finite")
        # Orbit spacing also limits displacement of the more distant support endpoint.
        radius = max(np.linalg.norm(p[:3, 3] - starts[primary][:3, 3]) for p in starts.values())
        offset = (
            starts[next(name for name in arms if name != primary)][:3, 3] - starts[primary][:3, 3]
        )
        if support_rise != 0 and (
            abs(offset[2] + support_rise) >= radius or np.linalg.norm(offset[:2]) < 1e-8
        ):
            raise ValueError("Support rise exceeds the preserved endpoint sphere")
        polar = (
            0
            if support_rise == 0
            else abs(math.asin((offset[2] + support_rise) / radius) - math.asin(offset[2] / radius))
        )
        count = max(len(poses) - 1, math.ceil((abs(support_orbit) + polar) * radius / 0.005))
        # Reuse SO(3) interpolation with a fixed sample count via fractional poses.
        poses = [intermediate_pose(starts[primary], goal, f) for f in np.linspace(0, 1, count + 1)]
    for index, pose in enumerate(poses[1:], 1):
        delta = pose @ reference_inverse
        for name, arm in arms.items():
            target = delta @ starts[name]
            if support_orbit is not None and name != primary:
                target = starts[name].copy()
                offset = starts[name][:3, 3] - starts[primary][:3, 3]
                if support_rise != 0:
                    distance = np.linalg.norm(offset)
                    z = offset[2] + support_rise * index / (len(poses) - 1)
                    offset[:2] *= math.sqrt(distance**2 - z**2) / np.linalg.norm(offset[:2])
                    offset[2] = z
                target[:3, 3] = (
                    pose[:3, 3]
                    + axis_rotation([0, 0, 1], support_orbit * index / (len(poses) - 1)) @ offset
                )
            result = arm.inverse(target, seed=paths[name][-1], attempts=1)
            q = np.asarray(result["q_rad"], dtype=float)
            if (
                not result["success"]
                or q.shape != paths[name][-1].shape
                or not np.isfinite(q).all()
                or np.max(np.abs(q - paths[name][-1])) > max_joint_step
            ):
                raise ValueError(f"{name}: unreachable/discontinuous coordinated waypoint {index}")
            paths[name].append(q)
    return {name: np.asarray(path) for name, path in paths.items()}


def intermediate_pose(start, goal, fraction):
    vector = rotation_error(goal[:3, :3], start[:3, :3])
    angle = np.linalg.norm(vector)
    pose = start.copy()
    pose[:3, 3] += fraction * (goal[:3, 3] - start[:3, 3])
    if angle > 1e-12:
        pose[:3, :3] = axis_rotation(vector / angle, angle * fraction) @ start[:3, :3]
    return pose


def assess_pair_path(arms, paths, maximum_error=0.003, rigid=True):
    """Check joint-interpolated relative tool frames, not just planned waypoints."""
    if len(arms) != 2 or set(paths) != set(arms):
        raise ValueError("Two corresponding arm paths required")
    if not math.isfinite(maximum_error) or maximum_error <= 0:
        raise ValueError("Positive finite pair tolerance required")
    if any(
        np.asarray(path).ndim != 2 or len(path) < 2 or not np.isfinite(path).all()
        for path in paths.values()
    ):
        raise ValueError("Finite paths with at least two waypoints required")
    names = list(arms)
    a, b = names
    reference = np.linalg.inv(arms[a].forward(paths[a][0])) @ arms[b].forward(paths[b][0])
    errors, angles, distances = [], [], []
    reference_distance = np.linalg.norm(reference[:3, 3])
    for fraction in np.linspace(0, 1, max(len(path) for path in paths.values()) * 4):
        first, second = [arms[n].forward(interpolate_joints(paths[n], fraction)) for n in names]
        relation = np.linalg.inv(first) @ second
        errors.append(
            np.linalg.norm(relation[:3, 3] - reference[:3, 3])
            if rigid
            else abs(np.linalg.norm(relation[:3, 3]) - reference_distance)
        )
        angles.append(np.linalg.norm(rotation_error(relation[:3, :3], reference[:3, :3])))
        distances.append(np.linalg.norm(first[:3, 3] - second[:3, 3]))
    return {
        "success": bool(
            max(errors) <= maximum_error
            and (not rigid or max(angles) <= 0.03)
            and min(distances) >= 0.12
        ),
        "maximum_relative_position_error_m": float(max(errors)),
        "constraint": "relative_frame" if rigid else "endpoint_distance",
        "maximum_relative_angle_error_rad": float(max(angles)),
        "minimum_tool_separation_m": float(min(distances)),
    }


def socket_poses(board, row, connector_from_tool, hover_height=0.16):
    """Housing underside seats on receptacle rim; pins extend 40 mm below it."""
    if not math.isfinite(hover_height) or hover_height < 0.06:
        raise ValueError("Hover must clear the 50 mm housing-to-pin-tip distance")
    center = np.asarray(board["origin_m"], dtype=float) + [
        row["x_mm"] / 1000,
        row["y_mm"] / 1000,
        0,
    ]
    seated = transform(center + [0, 0, 0.01], [0, 0, math.radians(row["yaw_deg"])])
    hover = seated.copy()
    hover[2, 3] = center[2] + hover_height
    relation = rigid_pose(connector_from_tool)
    return {"hover": hover @ relation, "seated": seated @ relation}
