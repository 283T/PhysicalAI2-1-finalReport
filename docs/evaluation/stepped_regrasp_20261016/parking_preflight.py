"""Nominal parking feasibility only; no release or seat stability experiment."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

sys.path.insert(0, "scripts")
from arm_collision_bounds import ArmCollisionBounds
from park_arm_plan import parking_plan
from plan_reach import load_arms

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
if args.output.exists():
    raise ValueError("Use a new output file")
base = Path("docs/evaluation/cooperative_nest_20261016")
nest_path = base / "rigid_midpoint_plan.json"
retreat_path = base / "top_regrasp_parked_left.json"
nest = json.loads(nest_path.read_text())
retreat = json.loads(retreat_path.read_text())
models = Path("docs/evaluation/stepped_regrasp_20261016")
world = base / "hd_wire_approach_fixture_rejected/case/world.sdf"
left = nest["attempts"][nest["selected_attempt"]]["phases"][-1]["joint_paths"]["left_arm"][-1]
right = retreat["attempts"][0]["transit_routes"][0]["path"][0]
row = {
    "sim_time_s": 0,
    "arms": {
        n: {"q_rad": q, "fingers": [0.018, 0.018]}
        for n, q in [("left_arm", left), ("right_arm", right)]
    },
}
arms = load_arms(world, models)
bounds = {n: ArmCollisionBounds(models / (n + ".urdf"), a) for n, a in arms.items()}
report = {
    "success": False,
    "scope": "Nominal endpoint assumptions from earlier offline plans, both arms assumed physically released and open. Candidate URDFs; world supplies bases and fixtures. No actual seat, release, parking or regrasp proof",
    "plans": [],
}
for moving in ["left_arm", "right_arm"]:
    plan = parking_plan(world, arms, bounds, row, moving)
    report["plans"].append(plan)
    if not plan["success"]:
        break
    row["arms"][moving]["q_rad"] = plan["paths"][moving][-1]
report["success"] = len(report["plans"]) == 2 and all(p["success"] for p in report["plans"])
report["input_sha256"] = {
    str(p): hashlib.sha256(p.read_bytes()).hexdigest()
    for p in [
        nest_path,
        retreat_path,
        world,
        models / "left_arm.urdf",
        models / "right_arm.urdf",
        Path("scripts/park_arm_plan.py"),
    ]
}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(report, indent=2) + "\n")
print(
    {
        "success": report["success"],
        "plans": [
            {k: p.get(k) for k in ["success", "moving_arm", "checks"]} for p in report["plans"]
        ],
    }
)
if not report["success"]:
    raise SystemExit(1)
