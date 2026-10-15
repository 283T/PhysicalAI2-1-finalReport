"""Offline 19 mm support candidate from archived RGB-D; never executes motion."""

import argparse
import gzip
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, "scripts")
from arm_collision_bounds import (
    ArmCollisionBounds,
    FingerStateBounds,
    finger_collision_intervals,
    screen_paths,
)
from cooperative_pick import shared_lift
from fixture_collision_bounds import FixtureBounds
from plan_reach import load_arms
from plan_wire_pickup import plan_wire_pickup

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
if args.output.exists():
    raise ValueError("Use a new output file")
root = Path("docs/evaluation/dual_pickup_20261016/support140_settle30_dual_hold/pickup")
models = Path("docs/evaluation/stepped_regrasp_20261016")
world = Path("docs/evaluation/cooperative_nest_20261016/fine_step_preclose_rejected/case/world.sdf")
observation_path = root / "suspended_wire_observation/wire_supports.json"
observation = json.loads(observation_path.read_text())
history_path = root / "trajectory.json.gz"
history = json.loads(gzip.decompress(history_path.read_bytes()))
row = min(history, key=lambda r: abs(r["sim_time_s"] - observation["sensor_stamp_ns"] / 1e9))
arms = load_arms(world, models)
raw = {n: ArmCollisionBounds(models / (n + ".urdf"), a) for n, a in arms.items()}
bounds = {
    n: FingerStateBounds(
        b,
        {
            n + "_finger_" + f + "_joint": finger_collision_intervals(row["arms"][n]["fingers"])[i]
            for i, f in enumerate(["left", "right"])
        },
    )
    for n, b in raw.items()
}
report = {
    "success": False,
    "scope": "Archived RGB-D and robot feedback, candidate permanent URDFs, static geometry only. World used for bases and fixtures. No fresh observation, closure, wire holding, loaded transfer or insertion proof",
    "source_robot_time_difference_s": abs(row["sim_time_s"] - observation["sensor_stamp_ns"] / 1e9),
}
try:
    plan = plan_wire_pickup(
        arms,
        bounds,
        world,
        observation,
        {n: r["q_rad"] for n, r in row["arms"].items()},
        "left_arm",
        "P3",
        include_lift=False,
        include_carrier=True,
        tip_depth_m=0.019,
        face_quarter_turns=2,
        route_transit=True,
        wire_profile=models / "left_arm.urdf",
    )
    report["support_plan"] = plan
    current = {n: np.array(r["q_rad"]) for n, r in row["arms"].items()}
    current["left_arm"] = np.array(plan["paths"]["approach"]["left_arm"][-1])
    _, paths = shared_lift(arms, current)
    closed = {
        n: FingerStateBounds(
            b, {n + "_finger_" + f + "_joint": [-0.0005, 0.0205] for f in ["left", "right"]}
        )
        for n, b in raw.items()
    }
    report["shared_lift"] = {
        "arms": screen_paths(closed, paths),
        "fixtures": FixtureBounds(world, include_carrier=True).screen(closed, paths, margin=0.001),
        "paths": {n: p.tolist() for n, p in paths.items()},
    }
    report["success"] = (
        plan["success"]
        and report["shared_lift"]["arms"]["clear"]
        and report["shared_lift"]["fixtures"]["clear"]
    )
except Exception as error:
    report.update(error=str(error), diagnostics=getattr(error, "diagnostics", None))
report["input_sha256"] = {
    str(p): hashlib.sha256(p.read_bytes()).hexdigest()
    for p in [
        world,
        observation_path,
        history_path,
        models / "left_arm.urdf",
        models / "right_arm.urdf",
    ]
}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(report, indent=2) + "\n")
print({k: v for k, v in report.items() if k not in ["support_plan", "shared_lift", "input_sha256"]})
if not report["success"]:
    raise SystemExit(1)
