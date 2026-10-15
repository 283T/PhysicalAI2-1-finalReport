"""Screen drawing-selected insertion after the assumed offline top regrasp."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, "scripts")
from arm_collision_bounds import ArmCollisionBounds
from plan_reach import load_arms
from top_socket_plan import plan_socket

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
if args.output.exists():
    raise ValueError("Use a new output file")
models = Path("docs/evaluation/stepped_regrasp_20261016")
case = Path("docs/evaluation/cooperative_nest_20261016/hd_hover_wall_limit/case")
world = case / "world.sdf"
top = json.loads((models / "validation.json").read_text())
if not top["success"]:
    raise ValueError("Matching offline top pickup plan required")
current = {n: np.zeros(6) for n in ["left_arm", "right_arm"]}
current["left_arm"] = np.asarray(top["paths"]["lift"]["left_arm"][-1])
arms = load_arms(world, models)
bounds = {n: ArmCollisionBounds(models / (n + ".urdf"), a) for n, a in arms.items()}
plan_path = Path(
    "docs/evaluation/dual_pickup_20261016/support140_settle30_dual_hold/pickup/connection_plan.json"
)
drawing = json.loads(plan_path.read_text())
if drawing["image_sha256"] != hashlib.sha256((case / "drawing.png").read_bytes()).hexdigest():
    raise ValueError("Drawing SHA mismatch")
report = plan_socket(
    arms,
    bounds,
    world,
    current,
    "left_arm",
    json.loads((case / "board.json").read_text()),
    drawing,
    "P3",
    top["connector_from_tool"],
)
report["scope_note"] = (
    "Uses the assumed top-pickup lift endpoint and parked right arm, not an actual regrasp. World supplies bases and fixtures; collision models are the permanent candidate URDFs. No physical insertion or wire support proof."
)
inputs = [
    world,
    case / "board.json",
    case / "drawing.png",
    models / "validation.json",
    models / "left_arm.urdf",
    models / "right_arm.urdf",
    plan_path,
    Path("scripts/top_socket_plan.py"),
    Path("scripts/carried_connector.py"),
]
report["input_sha256"] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(report, indent=2) + "\n")
print(
    {
        "success": report["success"],
        "attempts": [
            {k: v for k, v in a.items() if k in ["success", "reason", "checks"]}
            for a in report["attempts"]
        ],
    }
)
if not report["success"]:
    raise SystemExit(1)
