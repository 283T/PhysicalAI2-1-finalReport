"""Independently audit a finished preregistered pickup/hold trial, never insertion."""

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "scripts"))
from evidence_io import write_json_atomic
from evidence_stream import open_evidence
from replay_contact_evidence import payload_arm_audit
from replay_grip_evidence import replay


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit(registry_path, trial_id):
    registry = json.loads(registry_path.read_text())
    trial = next(t for t in registry["trials"] if t["trial_id"] == trial_id)
    root = ROOT / trial["output"]
    report_path, record_path = root / "pickup/validation.json", root / "recording.json"
    report, recording = json.loads(report_path.read_text()), json.loads(record_path.read_text())
    if report.get("trial_state") != "finished" or recording.get("trial_state") != "finished":
        raise ValueError("Only finished controller and recorder results may be audited")
    spec_path = ROOT / registry["acceptance_file"]
    spec = json.loads(spec_path.read_text())
    errors = []

    def require(condition, message):
        if not condition:
            errors.append(message)

    require(sha(spec_path) == registry["acceptance_sha256"], "Acceptance differs from registration")
    expected = {**registry["control_source_sha256"], **registry["input_sha256"]}
    # The acceptance JSON is the evaluator input, not a controller input.
    expected.pop(registry["acceptance_file"], None)
    for path, digest in expected.items():
        require(recording.get("source_sha256", {}).get(path) == digest,
                "Recorded source/input differs from registration: " + path)
    require(recording.get("changed_sources") == [], "Sources changed during execution")
    require(report.get("success") is True and recording.get("controller_exit_code") == 0,
            "Controller did not finish successfully")
    require(not any(report["parameters"].get(k) for k in (
        "place_in_nest", "after_nest_parking", "top_regrasp", "insert_after_regrasp")),
        "Unexpected task scope")
    require(report.get("world_sha256") == registry["input_sha256"][registry["inputs"]["world"]],
            "Physical report world differs from registration")

    preclose = report.get("primary_preclose_checks", [])
    require(bool(preclose) and preclose[-1].get("success") is True
            and preclose[-1].get("preferred_alignment") is True, "Final primary alignment failed")
    require(report.get("support_preclose", {}).get("success") is True,
            "Support alignment did not pass")
    limits = spec["holding"]
    holds = {"primary": report.get("primary_hold", {}),
             "left_arm": report.get("dual_holds", {}).get("left_arm", {}),
             "right_arm": report.get("dual_holds", {}).get("right_arm", {})}
    for name, hold in holds.items():
        require(hold.get("success") is True, name + ": hold did not pass")
        for key, threshold, lower in (
            ("minimum_lift_m", limits["minimum_lift_m_exclusive"], True),
            ("relative_slip_m", limits["maximum_position_slip_m_exclusive"], False),
            ("lift_slip_m", limits["maximum_position_slip_m_exclusive"], False),
            ("maximum_rotation_change_rad", limits["maximum_rotation_change_rad_exclusive"], False),
        ):
            value = hold.get(key)
            require(isinstance(value, (float, int)) and math.isfinite(value)
                    and (value > threshold if lower else value < threshold),
                    name + ": invalid/outside-limit " + key)
        require(hold.get("rotation_validated") is True, name + ": rotation not validated")

    windows = {}
    for phase in ("primary_hold", "dual_hold"):
        found = [w for w in report.get("dwell_windows", []) if w["phase"] == phase]
        require(len(found) == 1, phase + ": exactly one hold window required")
        if len(found) != 1:
            continue
        window = found[0]
        windows[phase] = window
        elapsed = window["end_sim_time_s"] - window["start_sim_time_s"]
        require(window.get("completed") is True and math.isfinite(elapsed)
                and elapsed >= limits["physical_dwell_duration_s"], phase + ": incomplete duration")
        require(math.isclose(window["elapsed_simulation_s"], elapsed, abs_tol=1e-10),
                phase + ": inconsistent duration metadata")
        require(window["distinct_observation_timestamps"] >= limits["minimum_distinct_timestamp_samples"],
                phase + ": insufficient distinct observations")

    observed_times = {"primary_hold": {"primary": set()},
                      "dual_hold": {"primary": set(), "support": set()}}
    out_of_window = [0]

    def observations(stream):
        for line in stream:
            row = json.loads(line)
            phase, grip = row["phase"], row["grip"]
            if phase in observed_times and grip in observed_times[phase]:
                stamp = row["sim_time_s"]
                observed_times[phase][grip].add(stamp)
                w = windows.get(phase)
                if w and not w["start_sim_time_s"] <= stamp <= w["end_sim_time_s"]:
                    out_of_window[0] += 1
            yield row

    with open_evidence(root / "pickup/grip_observations.jsonl.gz") as stream:
        grip_result = replay(report, observations(stream))
    require(grip_result["evidence_consistent"] and grip_result["all_recorded_grips_within_limits"],
            "Full grip replay did not pass")
    for phase, grips in observed_times.items():
        for grip, stamps in grips.items():
            require(len(stamps) >= limits["minimum_distinct_timestamp_samples"],
                    phase + "/" + grip + ": insufficient raw grip timestamps")
    require(out_of_window[0] == 0, "Raw observations outside recorded hold window")
    violations = payload_arm_audit(report)
    require(not violations and not report.get("unintended_contacts"), "Monitored contact violation")
    return {"scope": "New pickup/hold protocol only; not insertion, electrical continuity or uninstrumented contact proof. Video suitability is separate.",
            "series": registry.get("series_id", "B"), "trial_id": trial_id,
            "status": "passed" if not errors else "failed", "errors": errors,
            "controller_error": report.get("error"), "hold_results": holds, "hold_windows": windows,
            "grip_replay": grip_result,
            "raw_hold_timestamp_counts": {p: {g: len(v) for g, v in gs.items()}
                                           for p, gs in observed_times.items()},
            "contact_violations": violations, "capture_timing": recording.get("capture_timing"),
            "input_sha256": {"registry": sha(registry_path), "validation": sha(report_path),
                             "recording": sha(record_path), "auditor": sha(Path(__file__))}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, default=Path(__file__).with_name("registry.json"))
    parser.add_argument("--trial-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Preserve previous audit; use a new output")
    result = audit(args.registry, args.trial_id)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_json_atomic(args.output, result)
    print(result["status"], result["errors"])
    if result["status"] != "passed":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
