"""Continuous independent seat evaluation, including already connected plugs."""

import copy
import hashlib
import json
import time

import numpy as np
from arm_kinematics import rotation_error
from cooperative_transfer import rigid_pose
from project_paths import ROOT


def safe_observation(value):
    if isinstance(value, np.ndarray) and value.ndim == 0:
        return safe_observation(value.item())
    if isinstance(value, dict):
        return {k: safe_observation(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [safe_observation(v) for v in value]
    if isinstance(value, (float, np.floating)) and not np.isfinite(value):
        return None
    return value.item() if isinstance(value, np.generic) else value


def seat_observation(goal, reference, observed, criteria):
    try:
        goal = rigid_pose(goal)
        point = np.asarray(observed["plug"], dtype=float)
        first = np.asarray(reference["plug"], dtype=float)
        rotation = np.asarray(observed["plug_rotation_world"], dtype=float)
        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        rigid_pose(matrix)
        if point.shape != (3,) or first.shape != (3,) or not np.isfinite([point, first]).all():
            raise ValueError("Invalid position")
        values = {
            "lateral_error_m": float(np.linalg.norm(point[:2] - goal[:2, 3])),
            "height_error_m": float(abs(point[2] - goal[2, 3])),
            "angle_error_rad": float(np.linalg.norm(rotation_error(rotation, goal[:3, :3]))),
            "movement_m": float(np.linalg.norm(point - first)),
        }
    except (ValueError, KeyError, TypeError):
        return {"success": False, "reason": "Invalid seat pose observation"}
    limits = {
        "lateral_error_m": "maximum_lateral_error_m_exclusive",
        "height_error_m": "maximum_height_error_m_exclusive",
        "angle_error_rad": "maximum_angle_error_rad_exclusive",
        "movement_m": "maximum_movement_m_exclusive",
    }
    return {"success": all(values[k] < criteria[v] for k, v in limits.items()), **values}


def register_seat_guard(session, report, prefix, goal, extract, stream, *, kind):
    if kind not in ("passive_nest", "connected") or prefix in report.get("seat_references", {}):
        raise ValueError("New seat name and explicit passive_nest/connected kind required")
    path = ROOT / "configs/final_acceptance.json"
    data = path.read_bytes()
    criteria = json.loads(data)["connected_after_release"]
    goal = rigid_pose(goal).copy()
    row = session.tick()
    if row is None:
        raise RuntimeError("Fresh state required to begin seat monitoring")
    reference = copy.deepcopy(extract(row))
    if seat_observation(goal, reference, reference, criteria).get("reason"):
        raise RuntimeError("Invalid initial independent seat observation")
    report.setdefault("seat_references", {})[prefix] = {
        "kind": kind,
        "reference": copy.deepcopy(reference),
        "goal_pose": goal.tolist(),
        "criteria_sha256": hashlib.sha256(data).hexdigest(),
        "criteria": copy.deepcopy(criteria),
    }
    summary = {
        "samples": 0,
        "maximum_lateral_error_m": 0.0,
        "maximum_height_error_m": 0.0,
        "maximum_angle_error_rad": 0.0,
        "maximum_movement_m": 0.0,
    }
    report.setdefault("seat_guard_summaries", {})[prefix] = summary
    previous = [None]
    failed = [False]

    def guard(row):
        if failed[0]:
            raise RuntimeError(prefix + " seat failure is latched")
        stamp = row["sim_time_s"]
        if stamp == previous[0]:
            return
        observed = copy.deepcopy(extract(row))
        check = seat_observation(goal, reference, observed, criteria)
        if not np.isfinite(stamp) or previous[0] is not None and stamp < previous[0]:
            check = {"success": False, "reason": "Seat clock is invalid or reversed"}
        record_stamp = stamp if np.isfinite(stamp) else None
        previous[0] = stamp if record_stamp is not None else previous[0]
        summary["samples"] += 1
        for key in ("lateral_error_m", "height_error_m", "angle_error_rad", "movement_m"):
            summary["maximum_" + key] = max(summary["maximum_" + key], check.get(key, 0))
        if not check["success"]:
            failed[0] = True
            report.setdefault("seat_guard_failures", {})[prefix] = {
                "phase": row["phase"],
                "sim_time_s": record_stamp,
                **check,
            }
        stream.write(
            json.dumps(
                safe_observation(
                    {
                        "seat": prefix,
                        "phase": row["phase"],
                        "sim_time_s": record_stamp,
                        "observed": observed,
                        "check": check,
                    }
                ),
                allow_nan=False,
            )
            + "\n"
        )
        if not check["success"]:
            raise RuntimeError(prefix + " left its independently monitored seat")

    guard.has_failed = lambda: failed[0]
    guard.get_kind = lambda: kind
    session.evaluation_guards.append(guard)
    guard(row)
    return guard


def end_nest_guard_for_pickup(session, report, prefix, guard, grip_prefix, grip_guard):
    reference = report.get("seat_references", {}).get(prefix, {})
    if (
        reference.get("kind") != "passive_nest"
        or not hasattr(guard, "get_kind")
        or guard.get_kind() != "passive_nest"
        or prefix in report.get("seat_guard_failures", {})
        or not hasattr(guard, "has_failed")
        or guard.has_failed()
    ):
        raise RuntimeError("Only an intact passive nest monitor can end for pickup")
    if (
        guard not in session.evaluation_guards
        or grip_guard not in session.evaluation_guards
        or guard is grip_guard
        or grip_prefix not in report.get("grip_references", {})
    ):
        raise RuntimeError("An active separate new grip monitor is required before lifting")
    deadline = time.monotonic() + 2
    while True:
        row = session.tick()
        if (
            row is not None
            and report.get("grip_guard_summaries", {}).get(grip_prefix, {}).get("samples", 0) >= 1
        ):
            break
        if time.monotonic() >= deadline:
            raise RuntimeError("New grip must have a fresh evaluated observation")
    report.setdefault("seat_guard_endings", {})[prefix] = {
        "sim_time_s": row["sim_time_s"],
        "phase": row["phase"],
        "reason": "intentional physical pickup with active grip monitoring",
        "grip": grip_prefix,
    }
    session.evaluation_guards.remove(guard)
