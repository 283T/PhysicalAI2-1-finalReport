"""Pure geometry and acceptance criteria for the isolated contact benchmark."""

import numpy as np
from arm_kinematics import quaternion_rotation


def world_link_positions(poses, model, links):
    """PosePublisher / dynamic_pose link positions are relative to their model."""
    origin = np.asarray(poses[model]["xyz"])
    rotation = quaternion_rotation(poses[model]["q"])
    return {name: origin + rotation @ np.asarray(poses[name]["xyz"]) for name in links}


def assess_hold(baseline, samples, grip_reference):
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
    return {
        "success": rise > 0.07
        and tcp_rise > 0.07
        and slip < 0.005
        and lift_slip < 0.005
        and blockage > 0.0008,
        "minimum_lift_m": rise,
        "relative_slip_m": slip,
        "lift_slip_m": lift_slip,
        "minimum_tcp_lift_m": tcp_rise,
        "minimum_finger_travel_m": blockage,
        "hold_samples": len(samples),
    }
