"""Visible wire support proposals from registered RGB-D, never simulator poses.

The prototype branches have distinct insulation colours and an 8 mm square
cross-section. These are proposals for subsequent stable observation and path
checks, not evidence of a grasp or of free space around hidden cable sections.
"""

import math

import cv2
import numpy as np
from arm_kinematics import rotation_error


def visible_plane_points(points):
    """Exclude depth edges/adjacent faces; retain a dominant measured plane."""
    rng = np.random.default_rng(0)
    best = None
    for _ in range(32):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        mask = np.abs((points - a) @ (normal / norm)) < 0.0005
        if best is None or mask.sum() > best.sum():
            best = mask
    if best is None or best.sum() < max(24, 0.60 * len(points)):
        return None
    return points[best]


def propose_wire_supports(
    bgr, depth, intrinsics, calibration, connectors, *, preferred_distance_m=0.11
):
    if not math.isfinite(preferred_distance_m) or not 0.075 <= preferred_distance_m <= 0.20:
        raise ValueError("Preferred support distance must be finite and between 75 and 200 mm")
    if depth.shape != bgr.shape[:2]:
        raise ValueError("RGB and depth must be registered")
    matrix = np.asarray(intrinsics, dtype=float).reshape(3, 3)
    rotation = np.asarray(calibration["optical_to_world_rotation"], dtype=float)
    origin = np.asarray(calibration["position_m"], dtype=float)
    if (
        not np.isfinite(matrix).all()
        or matrix[0, 0] <= 0
        or matrix[1, 1] <= 0
        or rotation.shape != (3, 3)
        or origin.shape != (3,)
        or not np.isfinite(rotation).all()
        or not np.isfinite(origin).all()
        or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
        or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6)
    ):
        raise ValueError("Finite pinhole intrinsics and a proper camera rotation required")
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0]
    masks = {
        "P1": (hue < 15) | (hue > 170),
        "P2": (hue > 95) & (hue < 135),
        "P3": (hue > 18) & (hue < 40),
    }
    output = []
    for connector in connectors:
        plug = connector["plug_id"]
        if plug not in masks or "pose_6d" not in connector:
            continue
        center = np.asarray(connector["pose_6d"]["center_xyz_m"], dtype=float)
        if center.shape != (3,) or not np.isfinite(center).all():
            raise ValueError("Finite observed connector centre required")
        valid = (
            masks[plug]
            & (hsv[:, :, 1] > 100)
            & (hsv[:, :, 2] > 60)
            & np.isfinite(depth)
            & (depth > 0.1)
            & (depth < 2)
        )
        v, u = np.where(valid)
        z = depth[v, u].astype(float)
        optical = np.column_stack(
            ((u - matrix[0, 2]) * z / matrix[0, 0], (v - matrix[1, 2]) * z / matrix[1, 1], z)
        )
        points = optical @ rotation.T + origin
        distance = np.linalg.norm(points - center, axis=1)
        # Exclude the housing, junction and distant same-colour board sockets.
        eligible = (distance >= preferred_distance_m - 0.035) & (
            distance <= preferred_distance_m + 0.035
        )
        points, u, v = points[eligible], u[eligible], v[eligible]
        candidates = []
        for index in range(0, len(points), 4):
            near = np.linalg.norm(points - points[index], axis=1) < 0.014
            local = points[near]
            if len(local) < 24:
                continue
            source_count = len(local)
            local = visible_plane_points(local)
            if local is None:
                continue
            mean = local.mean(axis=0)
            _, singular, axes = np.linalg.svd(local - mean, full_matrices=False)
            if singular[1] < 1e-6 or singular[0] / singular[1] < 1.8:
                continue
            tangent, normal = axes[0].copy(), axes[2].copy()
            planar_rms = singular[2] / np.sqrt(len(local))
            along = (local - mean) @ tangent
            across = (local - mean) @ axes[1]
            length, width = np.ptp(along), np.ptp(across)
            if planar_rms > 0.0006 or not 0.005 <= width <= 0.011 or length < 0.020:
                continue
            if np.dot(normal, origin - mean) < 0:
                normal *= -1
            if np.dot(tangent, center - mean) < 0:
                tangent *= -1
            # Remove normal contamination and form a right-handed tool basis.
            tangent -= normal * np.dot(tangent, normal)
            tangent /= np.linalg.norm(tangent)
            surface = mean + axes[1] * (float(across.min() + across.max()) / 2)
            wire_center = surface - normal * 0.004
            tcp = np.eye(4)
            tcp[:3, :3] = np.column_stack((np.cross(normal, tangent), tangent, -normal))
            tcp[:3, 3] = wire_center - normal * 0.002
            candidates.append(
                {
                    "plug_id": plug,
                    "surface_xyz_m": surface.tolist(),
                    "wire_center_xyz_m": wire_center.tolist(),
                    "grasp_tcp_pose": tcp.tolist(),
                    "pixel_uv": [float(u[near].mean()), float(v[near].mean())],
                    "surface_normal_world": normal.tolist(),
                    "tangent_world": tangent.tolist(),
                    "observed_width_m": float(width),
                    "observed_length_m": float(length),
                    "planar_rms_m": float(planar_rms),
                    "points": len(local),
                    "source_points": source_count,
                    "distance_from_connector_m": float(np.linalg.norm(wire_center - center)),
                }
            )
        if candidates:
            best = min(
                candidates,
                key=lambda row: (
                    abs(row["distance_from_connector_m"] - preferred_distance_m)
                    + 10 * row["planar_rms_m"]
                ),
            )
            output.append(best)
    return output


class StableWireSupports:
    """Bound the whole three-frame window; reject missing or drifting proposals."""

    def __init__(self, required_ids):
        self.required_ids = frozenset(required_ids)
        if not self.required_ids or not self.required_ids <= {"P1", "P2", "P3"}:
            raise ValueError("Known nonempty wire IDs required")
        self.anchor = None
        self.count = 0

    def update(self, rows):
        if len(rows) != len(self.required_ids) or {r["plug_id"] for r in rows} != self.required_ids:
            self.anchor, self.count = None, 0
            return False
        poses = [
            np.asarray(r["grasp_tcp_pose"], dtype=float)
            for r in sorted(rows, key=lambda r: r["plug_id"])
        ]
        if any(
            pose.shape != (4, 4)
            or not np.isfinite(pose).all()
            or not np.allclose(pose[3], [0, 0, 0, 1])
            or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(pose[:3, :3]), 1, atol=1e-6)
            for pose in poses
        ):
            self.anchor, self.count = None, 0
            return False
        consistent = self.anchor is not None and all(
            np.linalg.norm(pose[:3, 3] - reference[:3, 3]) < 0.002
            and np.linalg.norm(rotation_error(pose[:3, :3], reference[:3, :3])) < 0.05
            for pose, reference in zip(poses, self.anchor)
        )
        if consistent:
            self.count += 1
        else:
            self.anchor, self.count = [pose.copy() for pose in poses], 1
        return self.count >= 3


def annotate_supports(bgr, rows):
    output = bgr.copy()
    for row in rows:
        pixel = tuple(round(value) for value in row["pixel_uv"])
        cv2.circle(output, pixel, 7, (255, 0, 255), 2)
        cv2.putText(
            output,
            row["plug_id"] + " wire",
            (pixel[0] + 10, pixel[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (255, 0, 255),
            1,
        )
    return output


def main():
    import argparse
    import hashlib
    import json
    from pathlib import Path

    from evidence_io import write_json_atomic

    parser = argparse.ArgumentParser(
        description="Replay a saved RGB-D observation for wire proposals"
    )
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    files = {
        name: args.source / name
        for name in (
            "camera.png",
            "depth.npz",
            "camera_intrinsics.json",
            "calibration.json",
            "detections.json",
        )
    }
    image = cv2.imread(str(files["camera.png"]))
    if image is None:
        raise ValueError("Saved camera image missing or unreadable")
    with np.load(files["depth.npz"]) as archive:
        rows = propose_wire_supports(
            image,
            archive["depth"],
            json.loads(files["camera_intrinsics.json"].read_text()),
            json.loads(files["calibration.json"].read_text()),
            json.loads(files["detections.json"].read_text())["connectors"],
        )
    args.output.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output / "wire_supports.png"), annotate_supports(image, rows))
    write_json_atomic(
        args.output / "wire_supports.json",
        {
            "source": "saved_registered_rgbd",
            "input_directory": str(args.source),
            "input_sha256": {
                name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in files.items()
            },
            "scope": "Single-frame proposals only; no accuracy, stability, reach or physical grasp validation",
            "wire_supports": rows,
        },
    )
    print(json.dumps({"proposed_wires": [row["plug_id"] for row in rows]}))


if __name__ == "__main__":
    main()
