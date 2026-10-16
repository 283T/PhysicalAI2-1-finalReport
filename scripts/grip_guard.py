"""Continuous independent grip evaluation, with a single closure reference."""

import copy
import json

from grasp_validation import assess_relative_grasp


def register_grip_guard(session, report, prefix, reference, extract, stream):
    if prefix in report.get("grip_references", {}):
        raise ValueError("New grip episode name required; preserve the original closure reference")
    reference = copy.deepcopy(reference)
    report.setdefault("grip_references", {})[prefix] = reference
    summary = {"samples": 0, "maximum_position_shift_m": 0.0, "maximum_angle_shift_rad": 0.0}
    report.setdefault("grip_guard_summaries", {})[prefix] = summary
    previous = [reference.get("time_s")]

    def guard(row):
        if row["sim_time_s"] == previous[0]:
            return
        previous[0] = row["sim_time_s"]
        observed = extract(row)
        check = assess_relative_grasp(reference, observed)
        summary["samples"] += 1
        for key in ("position_shift_m", "angle_shift_rad"):
            summary["maximum_" + key] = max(summary["maximum_" + key], check.get(key, 0.0))
        stream.write(
            json.dumps(
                {
                    "grip": prefix,
                    "phase": row["phase"],
                    "sim_time_s": row["sim_time_s"],
                    "observed": observed,
                    "check": check,
                }
            )
            + "\n"
        )
        if not check["success"]:
            report[prefix + "_guard_failure"] = {
                "phase": row["phase"],
                "sim_time_s": row["sim_time_s"],
                **check,
            }
            raise RuntimeError(prefix + " grip changed beyond the closure reference limits")

    session.evaluation_guards.append(guard)
    return guard


def end_grip_for_release(session, report, prefix, guard):
    """End this grip only for an explicit physical opening; preserve its evidence."""
    if guard not in session.evaluation_guards or prefix + "_guard_failure" in report:
        raise RuntimeError("Cannot release an inactive or failed monitored grip")
    row = session.tick()
    if row is None:
        raise RuntimeError("Fresh observation required before planned release")
    report.setdefault("grip_guard_endings", {})[prefix] = {
        "sim_time_s": row["sim_time_s"],
        "phase": row["phase"],
        "reason": "intentional physical opening",
    }
    session.evaluation_guards.remove(guard)
