"""Offline prepared-jaw comparison using the archived rejected wire approach."""

import argparse
import copy
import gzip
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, "scripts")
from arm_collision_bounds import ArmCollisionBounds, screen_paths, with_parked_arms
from fixture_collision_bounds import FixtureBounds
from gripper_model import measured_rest_gap
from plan_reach import load_arms
from plan_wire_pickup import plan_wire_pickup
from wire_preshape import closing_support_bounds, preshape_target

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
if args.output.exists():
    raise ValueError("Use a new output file")
root = Path("docs/evaluation/cooperative_nest_20261016/hd_wire_approach_fixture_rejected")
world = root / "case/world.sdf"
models = root / "models"
observation_path = root / "pickup/suspended_wire_observation/wire_supports.json"
observation = json.loads(observation_path.read_text())
history_path = root / "pickup/trajectory.json.gz"
history = json.loads(gzip.decompress(history_path.read_bytes()))
row = min(history, key=lambda r: abs(r["sim_time_s"] - observation["sensor_stamp_ns"] / 1e9))
arms = load_arms(world, models)
raw = {n: ArmCollisionBounds(models / (n + ".urdf"), a) for n, a in arms.items()}
target = preshape_target(
    measured_rest_gap(models / "left_arm.urdf", "left_arm_"),
    observation["wire_supports"][0]["observed_width_m"],
)
before, _ = closing_support_bounds(raw, row, "left_arm")
current = {n: r["q_rad"] for n, r in row["arms"].items()}
fixtures = FixtureBounds(world, include_carrier=True)
report = {
    "scope": "Offline only. Assumes empty-space preshape succeeded; actual motion, measured settling, fresh RGB-D, wire closure and holding are unproven",
    "target": target,
    "empty_space_preflight": {
        "arms": screen_paths(
            before, {n: np.repeat(np.asarray(q)[None, :], 2, axis=0) for n, q in current.items()}
        ),
        "fixtures": fixtures.screen(
            before,
            {n: np.repeat(np.asarray(q)[None, :], 2, axis=0) for n, q in current.items()},
            margin=0.001,
        ),
    },
}
prepared = copy.deepcopy(row)
# Worst allowed positive 0.5 mm settling error; the planning envelope adds
# another 0.5 mm and includes the entire subsequent stroke down to closed.
prepared["arms"]["left_arm"]["fingers"] = [target["target_travel_m"] + 0.0005] * 2
bounds, intervals = closing_support_bounds(raw, prepared, "left_arm")
report["closing_intervals_m"] = intervals
old = json.loads((root / "pickup/wire_plan.json").read_text())
parked = {n: q for n, q in current.items() if n != "left_arm"}
report["original_paths_with_prepared_bounds"] = {
    phase: {
        "arms": screen_paths(
            bounds, with_parked_arms({n: np.asarray(p) for n, p in paths.items()}, parked)
        ),
        "fixtures": fixtures.screen(
            bounds,
            with_parked_arms({n: np.asarray(p) for n, p in paths.items()}, parked),
            margin=0.001,
        ),
    }
    for phase, paths in old["paths"].items()
}
plan = plan_wire_pickup(
    arms,
    bounds,
    world,
    observation,
    current,
    "left_arm",
    "P3",
    include_lift=False,
    include_carrier=True,
    tip_depth_m=0.006,
    face_quarter_turns=2,
    route_transit=True,
)
report["new_plan"] = plan
report["success"] = plan["success"] and all(
    x["clear"] for x in report["empty_space_preflight"].values()
)
report["input_sha256"] = {
    str(p): hashlib.sha256(p.read_bytes()).hexdigest()
    for p in [
        world,
        observation_path,
        history_path,
        root / "pickup/wire_plan.json",
        models / "left_arm.urdf",
        models / "right_arm.urdf",
        Path("scripts/wire_preshape.py"),
    ]
}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(report, indent=2) + "\n")
print(
    {
        k: v
        for k, v in report.items()
        if k
        in [
            "success",
            "scope",
            "target",
            "empty_space_preflight",
            "original_paths_with_prepared_bounds",
        ]
    }
)
if not report["success"]:
    raise SystemExit(1)
