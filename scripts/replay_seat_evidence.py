"""Recompute passive/connected seat observations against frozen criteria."""

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from cooperative_transfer import rigid_pose
from evidence_io import write_json_atomic
from evidence_stream import open_evidence
from project_paths import ROOT
from seat_guard import seat_observation

METRICS = ("lateral_error_m", "height_error_m", "angle_error_rad", "movement_m")


def equal_check(actual, saved):
    if not isinstance(saved, dict) or actual.keys() != saved.keys():
        return False
    for key, value in actual.items():
        other = saved[key]
        if isinstance(value, (bool, str)):
            if type(value) is not type(other) or value != other:
                return False
        elif type(other) not in (int, float) or not math.isclose(
            value, other, rel_tol=1e-8, abs_tol=1e-10
        ):
            return False
    return True


def same_goal(actual, expected):
    try:
        return bool(np.allclose(rigid_pose(actual), rigid_pose(expected), atol=1e-9, rtol=0))
    except (ValueError, TypeError):
        return False


def replay(report, records, *, required_windows=None, expected_seats=None):
    frozen = (ROOT / "configs/final_acceptance.json").read_bytes()
    criteria = json.loads(frozen)["connected_after_release"]
    references = report.get("seat_references", {})
    requested = (
        required_windows
        if required_windows is not None
        else {name: "socket_after_release" for name in references if name == "P3_connected"}
    )
    errors, failures, previous, first_failures = [], [], {}, {}
    summaries = {
        name: {"samples": 0, **{"maximum_" + key: 0.0 for key in METRICS}} for name in references
    }
    windows = {
        name: {"phase": phase, "samples": 0, "all_observations_within_limits": True}
        for name, phase in requested.items()
    }
    window_references = {}
    endings = report.get("seat_guard_endings", {})
    if expected_seats is not None and set(expected_seats) != set(references):
        errors.append("Recorded seats differ from independently expected seats")
    if not set(requested) <= set(references):
        errors.append("Required window has no recorded seat reference")
    for name, ref in references.items():
        if ref.get("kind") not in ("passive_nest", "connected"):
            errors.append(name + ": invalid seat kind")
        if (
            ref.get("criteria_sha256") != hashlib.sha256(frozen).hexdigest()
            or ref.get("criteria") != criteria
        ):
            errors.append(name + ": frozen criteria provenance mismatch")
        if expected_seats is not None and ref.get("kind") != expected_seats.get(name, {}).get(
            "kind"
        ):
            errors.append(name + ": seat kind differs from independently expected seat")
        if expected_seats is not None and not same_goal(
            ref.get("goal_pose"), expected_seats.get(name, {}).get("goal_pose")
        ):
            errors.append(name + ": goal differs from independently expected seat")
    for index, record in enumerate(records):
        name, stamp = record["seat"], record["sim_time_s"]
        if name not in references:
            errors.append(f"Record {index}: unknown seat reference")
            continue
        ref = references[name]
        valid_time = type(stamp) in (int, float) and math.isfinite(stamp)
        old = previous.get(name)
        if (
            not valid_time
            or old is not None
            and stamp <= old
            or old is None
            and stamp != ref["reference"].get("time_s")
        ):
            errors.append(f"Record {index}: missing initial sample or invalid seat clock")
        if valid_time:
            previous[name] = stamp
        if record["observed"].get("time_s") != stamp:
            errors.append(f"Record {index}: observation clock mismatch")
        if name in endings and valid_time and stamp > endings[name]["sim_time_s"]:
            errors.append(f"Record {index}: observation follows seat monitor ending")
        check = seat_observation(ref["goal_pose"], ref["reference"], record["observed"], criteria)
        if not valid_time or old is not None and stamp < old:
            check = {"success": False, "reason": "Seat clock is invalid or reversed"}
        if not equal_check(check, record["check"]):
            errors.append(f"Record {index}: saved seat check differs from recomputation")
        summary = summaries[name]
        summary["samples"] += 1
        for key in METRICS:
            summary["maximum_" + key] = max(summary["maximum_" + key], check.get(key, 0))
        if not check["success"]:
            failure = {"phase": record["phase"], "sim_time_s": stamp, **check}
            failures.append({"seat": name, **failure})
            first_failures.setdefault(name, failure)
        if name in requested and record["phase"] == requested[name]:
            window = windows[name]
            if name not in window_references:
                window_references[name] = record["observed"]
                window["first_sim_time_s"] = stamp
            window["last_sim_time_s"] = stamp
            window["samples"] += 1
            local = seat_observation(
                ref["goal_pose"], window_references[name], record["observed"], criteria
            )
            window["all_observations_within_limits"] &= (
                check["success"] and local["success"] and valid_time
            )
    for name, summary in summaries.items():
        if not summary["samples"] or not equal_check(
            summary, report.get("seat_guard_summaries", {}).get(name)
        ):
            errors.append(name + ": summary/count does not match complete evidence")
    saved_failures = report.get("seat_guard_failures", {})
    if set(first_failures) != set(saved_failures) or any(
        not equal_check(failure, saved_failures.get(name))
        for name, failure in first_failures.items()
    ):
        errors.append("Saved seat failures do not match first recomputed failures")
    for name, ending in endings.items():
        ref = references.get(name, {})
        grip = ending.get("grip")
        grip_time = report.get("grip_references", {}).get(grip, {}).get("time_s")
        if (
            ref.get("kind") != "passive_nest"
            or name in first_failures
            or ending.get("reason") != "intentional physical pickup with active grip monitoring"
            or ending.get("sim_time_s") != previous.get(name)
            or grip_time is None
            or grip_time > ending.get("sim_time_s", -1)
            or report.get("grip_guard_summaries", {}).get(grip, {}).get("samples", 0) < 1
        ):
            errors.append(name + ": invalid seat ending or missing new grip")
    for name, window in windows.items():
        first, last = window.get("first_sim_time_s"), window.get("last_sim_time_s")
        valid = all(isinstance(t, (int, float)) and math.isfinite(t) for t in (first, last))
        window["duration_s"] = last - first if valid else None
        window["success"] = bool(
            valid
            and window["all_observations_within_limits"]
            and window["samples"] >= criteria["minimum_samples"]
            and last - first >= criteria["minimum_observation_duration_s"]
        )
    if not references:
        errors.append("No monitored seats; this trial did not reach seat monitoring")
    return {
        "scope": "Replays recorded seat observations only. Default goals come from recorded references; drawing agreement, release, withdrawal, native contact and sampling between records need separate evidence.",
        "evidence_consistent": not errors,
        "all_recorded_seats_within_limits": bool(references) and not errors and not failures,
        "all_requested_windows_within_limits": bool(windows)
        and not errors
        and all(w["success"] for w in windows.values()),
        "trial_success": report.get("success", False),
        "summaries": summaries,
        "windows": windows,
        "failed_observations": failures,
        "errors": errors,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pickup", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = json.loads((args.pickup / "validation.json").read_text())
    if report.get("trial_state") != "finished":
        parser.error("Only finished trials can be replayed")
    with open_evidence(args.pickup / "seat_observations.jsonl.gz") as stream:
        result = replay(report, (json.loads(line) for line in stream))
    write_json_atomic(args.output, result)
    print(json.dumps(result, indent=2))
    if not result["evidence_consistent"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
