"""Pure geometry and acceptance criteria for the isolated contact benchmark."""

import numpy as np
from arm_kinematics import quaternion_rotation, rotation_error
from gripper_model import MAX_TRAVEL_M, gap_m


def world_link_positions(poses, model, links):
    """PosePublisher / dynamic_pose link positions are relative to their model."""
    origin = np.asarray(poses[model]["xyz"])
    rotation = quaternion_rotation(poses[model]["q"])
    return {name: origin + rotation @ np.asarray(poses[name]["xyz"]) for name in links}


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
    contact_opening = bool(
        np.isfinite(fingers).all()
        and fingers.min() >= -0.0005
        and fingers.max() <= MAX_TRAVEL_M + 0.0005
        and np.max(np.abs(openings - housing_width)) < 0.001
    )
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
