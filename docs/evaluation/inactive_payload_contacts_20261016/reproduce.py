"""Re-audit three fixed-P3 failures for contact with non-active harness parts."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

sys.path.insert(0, "scripts")
from replay_contact_evidence import payload_arm_audit

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
if args.output.exists():
    raise ValueError("Preserve earlier audit")
root = Path("docs/evaluation/cooperative_nest_20261016")
reports = {}
for name in (
    "preshape_nest_seat_wall_limit",
    "stepped_descent_tracking_failure",
    "centred_unplanned_connector_contact",
):
    path = root / name / "pickup/validation.json"
    report = json.loads(path.read_text())
    if report["trial_state"] != "finished":
        raise ValueError("Only finished trials")
    violations = payload_arm_audit(report)
    reports[name] = {
        "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "trial_success": report["success"],
        "arm_contact_audit_passed": not violations,
        "violations": violations,
    }
result = {
    "scope": "Retrospective contact audit for the fixed-P3 sequence only; intended branch P3. Original failed results remain unchanged. Native duplicate message counts are not forces or independent contacts. Contact association does not by itself prove the sole cause of tracking error.",
    "active_plugs": ["P3"],
    "reports": reports,
    "source_sha256": {
        str(p): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (Path("scripts/arm_contacts.py"), Path("scripts/replay_contact_evidence.py"))
    },
    "final_acceptance_eligible": False,
}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps({name: len(value["violations"]) for name, value in reports.items()}))
