"""Independent fixed-P3 contact replay and payload/arm role audit."""

import argparse
import hashlib
import io
import json
from pathlib import Path

from arm_contacts import arm_fixture_pair, inter_arm_pair
from assembly_case import validate_connection_plan
from carried_contact_guard import CarriedContactGuard
from evidence_io import write_json_atomic
from evidence_stream import open_evidence


def payload_arm_audit(report):
    """Fixed P3 sequence only; all-connectors sequencing needs an explicit policy."""
    violations = []
    for record in report.get("contacts", []):
        a, b, phase = record["collision1"], record["collision2"], record["phase"]
        if inter_arm_pair(a, b) or arm_fixture_pair(a, b):
            violations.append({**record, "reason": "Native arm/arm or arm/fixture contact"})
            continue
        if phase == "final_open":
            continue  # Teardown is outside the retained contact-monitor interval.

        def is_plug(name):
            parts = name.split("::")
            return any(x == "wire_harness_multi" and y == "P3" for x, y in zip(parts, parts[1:]))

        other = b if is_plug(a) else a if is_plug(b) else None
        if other is None:
            continue
        owner = None
        if phase.startswith(("top_", "socket_")):
            owner = "left_arm"
        elif phase.startswith(
            (
                "primary_",
                "support_",
                "suspended_wire_",
                "nest_held_",
                "nest_transfer_",
                "nest_primary_",
            )
        ) or phase in ("shared_lift", "dual_hold", "nest_planning", "nest_before_release"):
            owner = "right_arm"
        parts = other.split("::")
        for arm in ("left_arm", "right_arm"):
            if arm not in parts:
                continue
            index = parts.index(arm)
            link = parts[index + 1] if index + 1 < len(parts) else None
            if arm != owner or link not in (arm + "_finger_left", arm + "_finger_right"):
                violations.append(
                    {
                        **record,
                        "reason": "P3 touched a non-gripping arm/link",
                        "expected_gripping_arm": owner,
                    }
                )
    return violations


def replay(report, records, expected_sockets):
    metadata = report.get("carried_contact_guards", {})
    errors, output, buffers, guards = [], {}, {}, {}
    native_counts, replay_counts = {}, {}
    for row in report.get("contacts", []):
        key = (row["phase"], *sorted((row["collision1"], row["collision2"])))
        count = row.get("messages")
        if key in native_counts or type(count) is not int or count <= 0:
            errors.append("Invalid or duplicate native contact aggregate")
        else:
            native_counts[key] = count
    if not expected_sockets or set(metadata) != set(expected_sockets):
        errors.append("Carried contact episodes differ from drawing-selected plugs")
    for plug, saved in metadata.items():
        if plug not in expected_sockets or saved.get("socket") != expected_sockets[plug]:
            errors.append(plug + ": socket differs from the validated drawing")
            continue
        buffers[plug] = io.StringIO()
        guards[plug] = CarriedContactGuard(
            plug, expected_sockets[plug], saved["start_sim_time_s"], output, buffers[plug]
        )
        boundary = saved.get("allow_target_from_sim_time_s")
        if boundary is not None:
            guards[plug].begin_descent(boundary)
    for index, record in enumerate(records):
        plug = record["plug"]
        if plug not in guards:
            errors.append(f"Record {index}: unknown carried connector")
            continue
        key = (record["phase"], *sorted((record["collision1"], record["collision2"])))
        replay_counts[key] = replay_counts.get(key, 0) + 1
        buffer = buffers[plug]
        buffer.seek(0)
        buffer.truncate()
        stamp = record["sim_time_s"]
        if stamp is None:
            stamp = float("nan")
        guards[plug].observe(record["collision1"], record["collision2"], stamp, record["phase"])
        generated = buffer.getvalue()
        if not generated or json.dumps(json.loads(generated), sort_keys=True) != json.dumps(
            record, sort_keys=True
        ):
            errors.append(f"Record {index}: native contact check differs from recomputation")
    if any(count > native_counts.get(key, 0) for key, count in replay_counts.items()):
        errors.append("Processed contacts are not covered by native contact aggregates")
    summaries = output.get("carried_contact_guards", {})
    for plug, saved in metadata.items():
        actual = summaries.get(plug, {}).get("summary")
        if json.dumps(actual, sort_keys=True) != json.dumps(saved.get("summary"), sort_keys=True):
            errors.append(plug + ": contact summary/count mismatch")
    if output.get("carried_contact_failures", []) != report.get("carried_contact_failures", []):
        errors.append("Recorded carried-contact failures differ from recomputation")
    target_passed = bool(guards)
    for guard in guards.values():
        try:
            guard.require_target_contact()
        except RuntimeError:
            target_passed = False
    violations = payload_arm_audit(report)
    return {
        "scope": "Fixed-P3 contact evidence only; does not prove drawing perception, seating, physical release/withdrawal or all-connectors completion. Phase-based arm ownership is specific to the current right-to-left regrasp sequence.",
        "evidence_consistent": not errors,
        "all_requested_target_contacts_recorded": not errors and target_passed,
        "arm_contact_audit_passed": not violations and not report.get("unintended_contacts"),
        "payload_arm_violations": violations,
        "recomputed_summaries": {p: v["summary"] for p, v in summaries.items()},
        "recomputed_failures": output.get("carried_contact_failures", []),
        "trial_success": report.get("success", False),
        "errors": errors,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("pickup", "case", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    report = json.loads((args.pickup / "validation.json").read_text())
    if report.get("trial_state") != "finished":
        parser.error("Only finished trials can be replayed")
    plan = json.loads((args.pickup / "connection_plan.json").read_text())
    rows = validate_connection_plan(plan, json.loads((args.case / "board.json").read_text()))
    if (
        plan.get("source") != "local_vision_model"
        or plan.get("image_sha256")
        != hashlib.sha256((args.case / "drawing.png").read_bytes()).hexdigest()
    ):
        parser.error("Matching local-AI drawing plan required")
    expected = {row["plug_id"]: row["socket"] for row in rows if row["plug_id"] == "P3"}
    with open_evidence(args.pickup / "carried_contacts.jsonl.gz") as stream:
        result = replay(report, (json.loads(line) for line in stream), expected)
    write_json_atomic(args.output, result)
    print(json.dumps(result, indent=2))
    if not result["evidence_consistent"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
