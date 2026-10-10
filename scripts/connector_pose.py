"""Known-face RGB-D pose estimation for coded prototype connector housings."""

import numpy as np

# Negative-X patches identify the outward local face normal.
FACE_CODES = {
    "z_positive": ("1 0 1 1", [0, 0, 1]),
    "z_negative": ("0 1 1 1", [0, 0, -1]),
    "y_positive": ("0 1 0 1", [0, 1, 0]),
    "y_negative": ("0.01 0.01 0.01 1", [0, -1, 0]),
}


def world_points(u, v, depth, intrinsics, calibration):
    distance = depth[v, u]
    fx, fy, cx, cy = intrinsics[0], intrinsics[4], intrinsics[2], intrinsics[5]
    optical = np.column_stack(((u - cx) * distance / fx, (v - cy) * distance / fy, distance))
    return (
        optical @ np.asarray(calibration["optical_to_world_rotation"]).T + calibration["position_m"]
    )


def estimate_pose(hsv, depth, interior, white, housing, intrinsics, calibration):
    valid = np.isfinite(depth) & (depth > 0.1) & (depth < 2)
    hue, saturation, value = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    codes = {
        "z_positive": (hue > 135) & (hue < 170) & (saturation > 110) & (value > 60),
        "z_negative": (hue > 75) & (hue < 100) & (saturation > 110) & (value > 60),
        "y_positive": (hue > 45) & (hue < 75) & (saturation > 110) & (value > 60),
        "y_negative": value < 35,
    }
    candidates = sorted(
        ((int((mask & interior & valid).sum()), name) for name, mask in codes.items()), reverse=True
    )
    count, face = candidates[0]
    if count < 5:
        return None
    v, u = np.where(interior & housing & valid)
    if len(u) < 60:
        return None
    points = world_points(u, v, depth, intrinsics, calibration)
    # Reject pin faces / a second visible side with a dominant-plane robust fit.
    rng = np.random.default_rng(0)
    best = None
    for _ in range(60):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        length = np.linalg.norm(normal)
        if length < 1e-9:
            continue
        normal /= length
        selected = np.abs((points - a) @ normal) < 0.0015
        if best is None or selected.sum() > best.sum():
            best = selected
    if best is None or best.sum() < max(50, 0.50 * len(points)):
        return None
    plane = points[best]
    midpoint = plane.mean(axis=0)
    _, _, axes = np.linalg.svd(plane - midpoint, full_matrices=False)
    normal = axes[-1]
    if np.dot(normal, np.asarray(calibration["position_m"]) - midpoint) < 0:
        normal = -normal
    residual = float(np.sqrt(np.mean(((plane - midpoint) @ normal) ** 2)))
    if residual > 0.001:
        return None

    # Select the face code and white marker on the fitted plane, not on an adjacent side.
    v_all, u_all = np.where(interior & valid)
    all_points = world_points(u_all, v_all, depth, intrinsics, calibration)
    same_face = np.zeros(depth.shape, dtype=bool)
    same_face[v_all, u_all] = np.abs((all_points - midpoint) @ normal) < 0.002
    candidates = sorted(
        ((int((mask & interior & valid & same_face).sum()), name) for name, mask in codes.items()),
        reverse=True,
    )
    count, face = candidates[0]
    if count < 5 or count < 1.8 * candidates[1][0]:
        return None

    def marker_center(mask):
        v, u = np.where(mask & interior & valid & same_face)
        if len(u) < 5:
            return None
        point = np.median(world_points(u, v, depth, intrinsics, calibration), axis=0)
        return point - normal * np.dot(point - midpoint, normal)

    positive, negative = marker_center(white), marker_center(codes[face])
    if positive is None or negative is None:
        return None
    x = positive - negative
    length = np.linalg.norm(x)
    if not 0.012 < length < 0.030:
        return None
    x /= length
    if face.startswith("z_"):
        z = normal if face == "z_positive" else -normal
        y = np.cross(z, x)
    else:
        y = normal if face == "y_positive" else -normal
        z = np.cross(x, y)
    rotation = np.column_stack((x, y, z))
    surface = (positive + negative) / 2
    center = surface - 0.010 * normal
    # Jaw motion across the short surface dimension; fingertips 2 mm above housing center.
    tool = np.column_stack((np.cross(normal, x), x, -normal))
    grasp = np.eye(4)
    grasp[:3, :3] = tool
    grasp[:3, 3] = center + 0.002 * normal
    return {
        "center_xyz_m": center.tolist(),
        "rotation_world_from_connector": rotation.tolist(),
        "visible_face": face,
        "surface_normal_world": normal.tolist(),
        "grasp_tcp_pose": grasp.tolist(),
        "plane_rms_m": residual,
        "plane_inliers": int(best.sum()),
        "face_code_pixels": count,
        "method": "Known face code + white X marker + RGB-D plane; ideal camera calibration",
    }
