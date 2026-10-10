"""Preflight straight TCP translations; these are not collision-aware trajectories."""

import numpy as np
from arm_kinematics import rotation_error


def translation_path(arm, start_q, target, spacing=0.01, max_joint_step=0.25):
    start_q = np.asarray(start_q, dtype=float)
    if start_q.ndim != 1 or not np.isfinite(start_q).all():
        raise ValueError("Start joints must be a finite vector")
    start = arm.forward(start_q)
    target = np.asarray(target, dtype=float)
    if target.shape != (4, 4) or not np.isfinite(target).all():
        raise ValueError("Target must be a finite 4x4 pose")
    if np.linalg.norm(rotation_error(target[:3, :3], start[:3, :3])) > 0.04:
        raise ValueError("Translation path requires an already aligned tool")
    if not np.isfinite([spacing, max_joint_step]).all() or spacing <= 0 or max_joint_step <= 0:
        raise ValueError("Path limits must be positive")
    steps = max(1, int(np.ceil(np.linalg.norm(target[:3, 3] - start[:3, 3]) / spacing)))
    path = [np.asarray(start_q, dtype=float)]
    for fraction in np.linspace(0, 1, steps + 1)[1:]:
        pose = target.copy()
        pose[:3, 3] = start[:3, 3] + fraction * (target[:3, 3] - start[:3, 3])
        result = arm.inverse(pose, seed=path[-1])
        q = np.asarray(result["q_rad"])
        if (
            not result["success"]
            or q.shape != start_q.shape
            or not np.isfinite(q).all()
            or np.max(np.abs(q - path[-1])) > max_joint_step
        ):
            raise ValueError("Translation has unreachable IK or a discontinuous joint step")
        path.append(q)
    return np.asarray(path)


def interpolate_joints(path, fraction):
    fraction = float(np.clip(fraction, 0, 1)) * (len(path) - 1)
    index = min(int(fraction), len(path) - 2)
    blend = fraction - index
    return path[index] + blend * (path[index + 1] - path[index])
