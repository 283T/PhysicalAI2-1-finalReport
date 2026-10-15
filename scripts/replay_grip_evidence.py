"""Recompute tool-relative slip from archived observations, including failures."""

import argparse
import json
import math
from pathlib import Path

from evidence_io import write_json_atomic
from evidence_stream import open_evidence
from grasp_validation import assess_relative_grasp


def replay(report, records):
    references = report.get("grip_references", {})
    errors = []
    summaries = {
        name: {"samples": 0, "maximum_position_shift_m": 0.0, "maximum_angle_shift_rad": 0.0}
        for name in references
    }
    previous, failures = {}, []
    endings = report.get("grip_guard_endings", {})
    for index, record in enumerate(records):
        name, stamp = record["grip"], record["sim_time_s"]
        if name not in references:
            errors.append(f"Record {index}: unknown closure reference {name}")
            continue
        if not math.isfinite(stamp) or stamp <= previous.get(name, references[name]["time_s"]):
            errors.append(f"Record {index}: non-increasing observation time")
        previous[name] = stamp
        if name in endings and stamp > endings[name]["sim_time_s"]:
            errors.append(f"Record {index}: grip observation follows its recorded release")
        if record["observed"]["time_s"] != stamp:
            errors.append(f"Record {index}: observation timestamp mismatch")
        check = assess_relative_grasp(references[name], record["observed"])
        saved = record["check"]
        if check.keys() != saved.keys() or any(
            (
                value != saved[key]
                if isinstance(value, (bool, str))
                else not math.isclose(value, saved[key], rel_tol=1e-8, abs_tol=1e-10)
            )
            for key, value in check.items()
        ):
            errors.append(f"Record {index}: recorded slip check differs from recomputation")
        summary = summaries[name]
        summary["samples"] += 1
        for key in ("position_shift_m", "angle_shift_rad"):
            summary["maximum_" + key] = max(summary["maximum_" + key], check.get(key, 0.0))
        if not check["success"]:
            failures.append({"grip": name, "phase": record["phase"], "sim_time_s": stamp, **check})
    for name, summary in summaries.items():
        expected = report.get("grip_guard_summaries", {}).get(name)
        if (
            not summary["samples"]
            or expected is None
            or summary.keys() != expected.keys()
            or any(
                not math.isclose(value, expected[key], rel_tol=1e-8, abs_tol=1e-10)
                for key, value in summary.items()
            )
        ):
            errors.append(name + ": summary/count does not match complete evidence")
    if not references:
        errors.append("No closure references; this trial did not reach guarded holding")
    for name, ending in endings.items():
        stamp = ending.get("sim_time_s")
        if (
            name not in references
            or not isinstance(stamp, (int, float))
            or not math.isfinite(stamp)
            or stamp < previous.get(name, float("inf"))
            or ending.get("reason") != "intentional physical opening"
        ):
            errors.append(name + ": invalid planned release boundary")
    return {
        "evidence_consistent": not errors,
        "all_recorded_grips_within_limits": bool(references) and not errors and not failures,
        "trial_success": report.get("success", False),
        "scope": "Replays recorded grip observations only; does not establish insertion or sampling completeness between records",
        "summaries": summaries,
        "intentional_release_boundaries": endings,
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
    with open_evidence(args.pickup / "grip_observations.jsonl.gz") as stream:
        result = replay(report, (json.loads(line) for line in stream))
    write_json_atomic(args.output, result)
    print(json.dumps(result, indent=2))
    if not result["evidence_consistent"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
