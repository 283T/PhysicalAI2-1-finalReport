"""Image-sized jaw preparation and conservative closing envelopes for wire pickup."""

import math

import numpy as np
from arm_collision_bounds import FingerStateBounds, finger_collision_intervals
from gripper_model import OPEN_TRAVEL_M


def preshape_target(rest_gap_m, observed_width_m):
    if (
        not all(map(math.isfinite, (rest_gap_m, observed_width_m)))
        or not 0 < rest_gap_m < 0.008
        or not 0.005 <= observed_width_m <= 0.011
    ):
        raise ValueError(
            "Bare jaws closing around the known 8 mm wire and valid measured width required"
        )
    width = max(0.008, observed_width_m)
    gap = width + 0.010
    travel = (gap - rest_gap_m) / 2
    if not 0 < travel < OPEN_TRAVEL_M:
        raise ValueError("Prepared opening must stay within normal jaw travel")
    return {
        "model_wire_width_m": 0.008,
        "observed_wire_width_m": observed_width_m,
        "sizing_width_m": width,
        "clearance_m": 0.010,
        "target_gap_m": gap,
        "target_travel_m": travel,
    }


def assess_preshape(fingers, rest_gap_m, target):
    finger_collision_intervals(fingers)
    values = np.asarray(fingers, dtype=float)
    error = float(np.max(np.abs(values - target["target_travel_m"])))
    gap = float(rest_gap_m + sum(values))
    return {
        "success": error < 0.0005 and gap - target["sizing_width_m"] >= 0.008,
        "maximum_travel_error_m": error,
        "measured_gap_m": gap,
        "remaining_wire_clearance_m": gap - target["sizing_width_m"],
        "scope": "Empty-space jaw preparation only; no wire closure or holding proof",
    }


def closing_support_bounds(bounds, row, support):
    if support not in bounds or set(bounds) != set(row["arms"]):
        raise ValueError("Both measured arm/finger states required")
    result, intervals = {}, {}
    for name, bound in bounds.items():
        while isinstance(bound, FingerStateBounds):
            bound = bound.bounds
        limits = finger_collision_intervals(row["arms"][name]["fingers"])
        if name == support:
            limits[:, 0] = -0.0005
        intervals[name] = {
            name + "_finger_" + side + "_joint": limits[i].tolist()
            for i, side in enumerate(["left", "right"])
        }
        result[name] = FingerStateBounds(bound, intervals[name])
    return result, intervals
