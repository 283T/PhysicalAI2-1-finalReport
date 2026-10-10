"""Colored connector housing + orientation marker detection from registered RGB-D only."""

import math

import cv2
import numpy as np
from connector_pose import estimate_pose


def detect_connectors(bgr, depth, intrinsics, calibration):
    if depth.shape != bgr.shape[:2]:
        raise ValueError("RGB and depth must be registered at the same resolution")
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # Distinct prototype connector colors; no simulator pose or case JSON is used.
    masks = {
        "P1": ((hsv[:, :, 0] < 15) | (hsv[:, :, 0] > 170)),
        "P2": (hsv[:, :, 0] > 95) & (hsv[:, :, 0] < 135),
        "P3": (hsv[:, :, 0] > 18) & (hsv[:, :, 0] < 40),
    }
    rotation = np.asarray(calibration["optical_to_world_rotation"], dtype=float)
    origin = np.asarray(calibration["position_m"], dtype=float)
    fx, fy, cx, cy = intrinsics[0], intrinsics[4], intrinsics[2], intrinsics[5]
    if not all(math.isfinite(value) for value in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
        raise ValueError("Camera focal lengths must be positive and finite")
    output = []
    debug = bgr.copy()
    for plug, hue_mask in masks.items():
        mask = (hue_mask & (hsv[:, :, 1] > 100) & (hsv[:, :, 2] > 60)).astype(np.uint8) * 255
        # Fill marker holes before opening, so the white patch does not bias the housing center.
        outlines, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(mask, outlines, -1, 255, cv2.FILLED)
        # Remove narrow colored wire branches while retaining the wider housing.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            if cv2.contourArea(contour) < 80:
                continue
            rect = cv2.minAreaRect(contour)
            (u, v), (width, height), _ = rect
            if min(width, height) < 7 or not 1.35 < max(width, height) / min(width, height) < 3.2:
                continue
            polygon = cv2.boxPoints(rect)
            region = np.zeros(mask.shape, np.uint8)
            cv2.fillConvexPoly(region, np.round(polygon).astype(np.int32), 255)
            interior = cv2.erode(region, np.ones((3, 3), np.uint8)) > 0
            white = interior & (hsv[:, :, 1] < 45) & (hsv[:, :, 2] > 170)
            wy, wx = np.where(white)
            if len(wx) < 3:
                continue  # Reject ambiguous 180-degree orientation.
            edges = np.roll(polygon, -1, axis=0) - polygon
            axis = edges[np.argmax(np.linalg.norm(edges, axis=1))].astype(float)
            axis /= np.linalg.norm(axis)
            marker = np.array([wx.mean() - u, wy.mean() - v])
            if abs(np.dot(axis, marker)) < 1.5:
                continue
            if np.dot(axis, marker) < 0:
                axis *= -1
            valid = interior & np.isfinite(depth) & (depth > 0.1) & (depth < 2)
            if valid.sum() < 30:
                continue
            distance = float(np.median(depth[valid]))
            optical = np.array([(u - cx) * distance / fx, (v - cy) * distance / fy, distance])
            surface = rotation @ optical + origin
            direction = rotation @ np.array([axis[0] / fx, axis[1] / fy, 0.0])
            yaw = math.atan2(direction[1], direction[0])
            # This is the observed top surface; gripper approach uses this point.
            candidates.append(
                {
                    "plug_id": plug,
                    "surface_xyz_m": surface.tolist(),
                    "yaw_rad": yaw,
                    "pixel_uv": [float(u), float(v)],
                    "depth_m": distance,
                    "marker_pixels": int(len(wx)),
                }
            )
            pose = estimate_pose(
                hsv,
                depth,
                interior,
                white,
                hue_mask & (hsv[:, :, 1] > 100) & (hsv[:, :, 2] > 60),
                intrinsics,
                calibration,
            )
            if pose is not None:
                candidates[-1]["pose_6d"] = pose
            cv2.polylines(debug, [np.round(polygon).astype(np.int32)], True, (0, 255, 0), 2)
            end = np.round(np.array([u, v]) + axis * 25).astype(int)
            cv2.arrowedLine(debug, (round(u), round(v)), tuple(end), (255, 255, 255), 2)
            cv2.putText(
                debug,
                plug,
                (round(u) + 10, round(v) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )
        # Multiple plausible housings of one color are ambiguous, never choose silently.
        if len(candidates) == 1:
            output.append(candidates[0])
    return output, debug
