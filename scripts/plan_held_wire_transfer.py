"""Offline nest-path search from a finished dual pickup, with observed joints."""

import argparse
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np
from arm_collision_bounds import ArmCollisionBounds, FingerStateBounds
from evidence_io import write_json_atomic
from plan_nest_transfer import plan_to_nest
from plan_reach import load_arms


def plan(world, models, pickup):
    validation = json.loads((pickup / "validation.json").read_text())
    if validation.get("trial_state") != "finished" or not validation.get("success"):
        raise ValueError("A finished successful dual pickup is required")
    if hashlib.sha256(world.read_bytes()).hexdigest() != validation["world_sha256"]:
        raise ValueError("World differs from executed trial")
    for name, digest in validation["model_sha256"].items():
        if hashlib.sha256((models / (name + ".urdf")).read_bytes()).hexdigest() != digest:
            raise ValueError("Robot model differs from executed trial")
    history = json.loads(gzip.decompress((pickup / "trajectory.json.gz").read_bytes()))
    row = next(r for r in reversed(history) if r["phase"] == "dual_hold")
    arms = load_arms(world, models)
    intervals = {
        name: {
            name + "_finger_" + side + "_joint": [max(-0.0005, q - 0.0005), min(0.0205, q + 0.0005)]
            for side, q in zip(("left", "right"), state["fingers"])
        }
        for name, state in row["arms"].items()
    }
    bounds = {
        name: FingerStateBounds(ArmCollisionBounds(models / (name + ".urdf"), arm), intervals[name])
        for name, arm in arms.items()
    }
    current = {name: np.array(state["q_rad"]) for name, state in row["arms"].items()}
    result = plan_to_nest(
        arms,
        bounds,
        world,
        current,
        "right_arm",
        np.array(validation["primary_connector_from_tool"]),
        include_carrier=True,
    )
    result.update(
        starting_sim_time_s=row["sim_time_s"],
        starting_joints_rad={name: q.tolist() for name, q in current.items()},
        finger_intervals_m=intervals,
        provenance={
            "world_sha256": validation["world_sha256"],
            "model_sha256": validation["model_sha256"],
            "pickup_validation_sha256": hashlib.sha256(
                (pickup / "validation.json").read_bytes()
            ).hexdigest(),
            "pickup_trajectory_sha256": hashlib.sha256(
                (pickup / "trajectory.json.gz").read_bytes()
            ).hexdigest(),
        },
        scope_note="Saved robot feedback and original image-derived connector/tool relation; fixed fixture geometry. Offline screening only, not physical transport. Execution requires new observations and runtime finger interval monitoring.",
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("world", "models", "pickup", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    result = plan(args.world, args.models, args.pickup)
    write_json_atomic(args.output, result)
    print(json.dumps({"success": result["success"], "attempts": len(result["attempts"])}, indent=2))


if __name__ == "__main__":
    main()
