"""Pure geometry and acceptance criteria for the isolated contact benchmark."""

import numpy as np
from arm_kinematics import quaternion_rotation, rotation_error
from gripper_model import MAX_TRAVEL_M, gap_m


def world_link_positions(poses, model, links):
    """PosePublisher / dynamic_pose link positions are relative to their model."""
    origin = np.asarray(poses[model]["xyz"])
    rotation = quaternion_rotation(poses[model]["q"])
    return {name: origin + rotation @ np.asarray(poses[name]["xyz"]) for name in links}


def assess_seat(samples, center=(0, 0, 0.23), rotation=None):
    """Require a stable, upright seat after opening and withdrawing the fingers."""
    if len(samples) < 10:
        return {"success": False, "reason": "Not enough seat observations"}
    points = np.asarray([s["plug"] for s in samples], dtype=float)
    times = np.asarray([s["time_s"] for s in samples], dtype=float)
    rotations = [np.asarray(s.get("plug_rotation_world"), dtype=float) for s in samples]
    reference = np.eye(3) if rotation is None else np.asarray(rotation, dtype=float)
    valid = (
        points.shape == (len(samples), 3)
        and np.isfinite(points).all()
        and np.isfinite(times).all()
        and np.all(np.diff(times) > 0)
        and all(
            r.shape == (3, 3)
            and np.isfinite(r).all()
            and np.allclose(r.T @ r, np.eye(3), atol=1e-6)
            and np.isclose(np.linalg.det(r), 1)
            for r in [reference, *rotations]
        )
    )
    if not valid:
        return {"success": False, "reason": "Invalid or missing seat pose observations"}
    lateral = float(np.max(np.linalg.norm(points[:, :2] - np.asarray(center)[:2], axis=1)))
    height = float(np.max(np.abs(points[:, 2] - center[2])))
    angle = float(max(np.linalg.norm(rotation_error(r, reference)) for r in rotations))
    movement = float(np.max(np.linalg.norm(points - points[0], axis=1)))
    duration = float(times[-1] - times[0])
    return {
        "success": lateral < 0.002
        and height < 0.001
        and angle < 0.05
        and movement < 0.001
        and duration >= 1.0,
        "maximum_lateral_error_m": lateral,
        "maximum_height_error_m": height,
        "maximum_angle_error_rad": angle,
        "maximum_movement_m": movement,
        "duration_s": duration,
        "samples": len(samples),
    }


def assess_relative_grasp(reference, observation):
    """Measure slip in the moving tool frame, allowing intentional world rotation."""
    points, rotations = [], []
    for row in (reference, observation):
        plug, tcp = np.asarray(row["plug"]), np.asarray(row["tcp"])
        matrices = [
            np.asarray(row.get(key), dtype=float)
            for key in ("plug_rotation_world", "tcp_rotation_world")
        ]
        if (
            plug.shape != (3,)
            or tcp.shape != (3,)
            or not np.isfinite([plug, tcp]).all()
            or any(
                r.shape != (3, 3)
                or not np.isfinite(r).all()
                or not np.allclose(r.T @ r, np.eye(3), atol=1e-6)
                or not np.isclose(np.linalg.det(r), 1)
                for r in matrices
            )
        ):
            return {"success": False, "reason": "Invalid tool-relative pose observations"}
        body, tool = matrices
        points.append(tool.T @ (plug - tcp))
        rotations.append(tool.T @ body)
    displacement = float(np.linalg.norm(points[1] - points[0]))
    angle = float(np.linalg.norm(rotation_error(rotations[1], rotations[0])))
    return {
        "success": displacement < 0.005 and angle < 0.05,
        "position_shift_m": displacement,
        "angle_shift_rad": angle,
    }


def assess_opening(fingers, housing_width=0.020):
    """Reject empty closure or a jammed wide opening using measured joint travel."""
    values = np.asarray(fingers, dtype=float)
    if (
        values.ndim != 2
        or values.shape[1] != 2
        or len(values) == 0
        or not np.isfinite(values).all()
        or not np.isfinite(housing_width)
        or housing_width <= 0
        or values.min() < -0.0005
        or values.max() > MAX_TRAVEL_M + 0.0005
    ):
        return {"success": False, "reason": "Invalid measured finger travel or housing width"}
    openings = gap_m(values.mean(axis=1))
    return {
        "success": bool(np.max(np.abs(openings - housing_width)) < 0.001),
        "minimum_jaw_opening_m": float(openings.min()),
        "maximum_jaw_opening_m": float(openings.max()),
        "expected_housing_width_m": housing_width,
        "scope": "Joint opening only; not proof of housing contact or stable grasp",
    }


def assess_hold(baseline, samples, grip_reference, housing_width=0.020):
    """Reject stationary objects, empty closure and slip during the hold window."""
    if len(samples) < 10:
        return {"success": False, "reason": "Not enough hold observations"}
    points = np.array([s["plug"] for s in samples])
    relative = np.array([np.array(s["plug"]) - s["tcp"] for s in samples])
    reference = np.array(grip_reference["plug"]) - grip_reference["tcp"]
    rise = float(np.min(points[:, 2]) - baseline["plug"][2])
    slip = float(np.max(np.linalg.norm(relative - relative[0], axis=1)))
    lift_slip = float(np.max(np.linalg.norm(relative - reference, axis=1)))
    tcp_rise = float(min(s["tcp"][2] for s in samples) - grip_reference["tcp"][2])
    blockage = min(min(s["fingers"]) for s in samples)
    fingers = np.asarray([s["fingers"] for s in samples])
    openings = gap_m(fingers.mean(axis=1))
    # Off-center housings stop the two fingers at different travels. Their sum
    # determines the actual opening, which must match the known short-side width.
    contact_opening = assess_opening(fingers, housing_width)["success"]
    rotation_validated = "plug_rotation_world" in grip_reference
    maximum_rotation = None
    rotation_ok = True
    if rotation_validated:
        matrices = [
            np.asarray(s.get("plug_rotation_world"), dtype=float)
            for s in [grip_reference, *samples]
        ]
        valid = all(
            matrix.shape == (3, 3)
            and np.isfinite(matrix).all()
            and np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-6)
            and np.isclose(np.linalg.det(matrix), 1)
            for matrix in matrices
        )
        if valid:
            maximum_rotation = float(
                max(np.linalg.norm(rotation_error(matrix, matrices[0])) for matrix in matrices[1:])
            )
        rotation_ok = bool(valid and maximum_rotation < 0.05)
    return {
        "success": rise > 0.07
        and tcp_rise > 0.07
        and slip < 0.005
        and lift_slip < 0.005
        and contact_opening
        and rotation_ok,
        "minimum_lift_m": rise,
        "relative_slip_m": slip,
        "lift_slip_m": lift_slip,
        "minimum_tcp_lift_m": tcp_rise,
        "minimum_finger_travel_m": blockage,
        "minimum_jaw_opening_m": float(openings.min()),
        "maximum_jaw_opening_m": float(openings.max()),
        "expected_housing_width_m": housing_width,
        "hold_samples": len(samples),
        "rotation_validated": rotation_validated,
        "maximum_rotation_change_rad": maximum_rotation,
    }
