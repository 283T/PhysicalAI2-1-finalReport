"""Visual-only connector end tags and RGB-D pose estimation for diagnostics."""

import cv2
import numpy as np
from build_world import box
from connector_pose import world_points

TAG_SIZE_M = 0.016
END_OFFSET_M = 0.02035


def dictionary():
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def marker_image(marker_id, pixels=120):
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary(), marker_id, pixels)
    return cv2.aruco.drawMarker(dictionary(), marker_id, pixels)


def connector_from_marker(marker_id):
    if marker_id not in range(6):
        raise ValueError("Unknown connector end tag")
    sign = 1 if marker_id % 2 == 0 else -1
    pose = np.eye(4)
    pose[:3, :3] = np.array([[0, 0, sign], [sign, 0, 0], [0, 1, 0]])
    pose[0, 3] = sign * END_OFFSET_M
    return pose


def add_end_markers(terminal, plug):
    if plug not in ("P1", "P2", "P3"):
        raise ValueError("Unknown prototype plug")
    first = 2 * (int(plug.removeprefix("P")) - 1)
    for marker_id in (first, first + 1):
        sign = 1 if marker_id % 2 == 0 else -1
        box(
            terminal,
            f"end_tag_{marker_id}_white",
            ".0002 .019 .019",
            f"{sign * 0.0202} 0 0 0 0 0",
            "1 1 1 1",
            collision=False,
        )
        cells = marker_image(marker_id, 6)
        for row, column in zip(*np.where(cells == 0)):
            y = sign * (column + 0.5 - 3) * TAG_SIZE_M / 6
            z = (3 - row - 0.5) * TAG_SIZE_M / 6
            box(
                terminal,
                f"end_tag_{marker_id}_{row}_{column}",
                f".0001 {TAG_SIZE_M / 6} {TAG_SIZE_M / 6}",
                f"{sign * 0.02035} {y} {z} 0 0 0",
                ".01 .01 .01 1",
                collision=False,
            )


def pose_from_corners(corners, depth, intrinsics, calibration, marker_id):
    """Fit the depth plane and intersect corner rays; no simulator poses or PnP branch."""
    corners = np.asarray(corners, dtype=float).reshape(4, 2)
    fx, fy, cx, cy = intrinsics[0], intrinsics[4], intrinsics[2], intrinsics[5]
    if (
        not np.isfinite(corners).all()
        or not np.isfinite([fx, fy, cx, cy]).all()
        or min(fx, fy) <= 0
    ):
        return None
    region = np.zeros(depth.shape, np.uint8)
    cv2.fillConvexPoly(region, np.round(corners).astype(np.int32), 255)
    interior = cv2.erode(region, np.ones((3, 3), np.uint8)) > 0
    valid = interior & np.isfinite(depth) & (depth > 0.1) & (depth < 2)
    v, u = np.where(valid)
    if len(u) < 25:
        return None
    points = world_points(u, v, depth, intrinsics, calibration)
    midpoint = points.mean(axis=0)
    _, _, axes = np.linalg.svd(points - midpoint, full_matrices=False)
    normal = axes[-1]
    residual = float(np.sqrt(np.mean(((points - midpoint) @ normal) ** 2)))
    if residual > 0.001:
        return None
    rays = (
        np.column_stack(((corners[:, 0] - cx) / fx, (corners[:, 1] - cy) / fy, np.ones(4)))
        @ np.asarray(calibration["optical_to_world_rotation"]).T
    )
    origin = np.asarray(calibration["position_m"])
    denominator = rays @ normal
    if np.any(np.abs(denominator) < 1e-6):
        return None
    distances = ((midpoint - origin) @ normal) / denominator
    if not np.isfinite(distances).all() or min(distances) <= 0:
        return None
    measured = origin + distances[:, None] * rays
    half = TAG_SIZE_M / 2
    template = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]])
    centre = measured.mean(axis=0)
    left, _, right = np.linalg.svd(template.T @ (measured - centre))
    orientation = right.T @ np.diag([1, 1, np.linalg.det(right.T @ left.T)]) @ left.T
    corner_error = float(
        np.max(np.linalg.norm(template @ orientation.T + centre - measured, axis=1))
    )
    if corner_error > 0.0015:
        return None
    marker = np.eye(4)
    marker[:3, :3], marker[:3, 3] = orientation, centre
    connector = marker @ np.linalg.inv(connector_from_marker(marker_id))
    return {
        "center_xyz_m": connector[:3, 3].tolist(),
        "rotation_world_from_connector": connector[:3, :3].tolist(),
        "marker_id": marker_id,
        "plane_rms_m": residual,
        "plane_inliers": len(points),
        "maximum_corner_fit_error_m": corner_error,
        "method": "Known 16 mm ArUco end tag, RGB-D plane and corner-ray intersections; ideal calibration",
    }


def detect_end_markers(bgr, depth, intrinsics, calibration):
    if depth.shape != bgr.shape[:2]:
        raise ValueError("Registered RGB-D is required")
    params = (
        # OpenCV 4.6 exposes the type but requires its factory for initialization.
        cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, "ArucoDetector")
        else cv2.aruco.DetectorParameters_create()
    )
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 3
    # Upsampling helps threshold small rendered tags; corners are returned in
    # original pixels, so CameraInfo and RGB-D coordinates remain unchanged.
    image = cv2.resize(bgr, None, fx=2, fy=2, interpolation=cv2.INTER_LINEAR)
    if hasattr(cv2.aruco, "ArucoDetector"):
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary(), params).detectMarkers(image)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary(), parameters=params)
    corners = [quad / 2 for quad in corners]
    debug = bgr.copy()
    candidates = {}
    counts = {}
    if ids is None:
        return [], debug
    cv2.aruco.drawDetectedMarkers(debug, corners, ids)
    for quad, marker_id in zip(corners, ids.flatten()):
        marker_id = int(marker_id)
        if marker_id not in range(6):
            continue
        plug = "P" + str(marker_id // 2 + 1)
        counts[plug] = counts.get(plug, 0) + 1
        candidates.setdefault(plug, [])
        pose = pose_from_corners(quad, depth, intrinsics, calibration, marker_id)
        if pose is not None:
            rotation = np.asarray(pose["rotation_world_from_connector"])
            center = np.asarray(pose["center_xyz_m"])
            candidates[plug].append(
                {
                    "plug_id": plug,
                    "surface_xyz_m": (
                        center + rotation @ connector_from_marker(marker_id)[:3, 3]
                    ).tolist(),
                    "yaw_rad": float(np.arctan2(rotation[1, 0], rotation[0, 0])),
                    "pixel_uv": np.asarray(quad).reshape(4, 2).mean(axis=0).tolist(),
                    "pose_6d": pose,
                }
            )
    return [
        rows[0] for plug, rows in candidates.items() if counts[plug] == 1 and len(rows) == 1
    ], debug
