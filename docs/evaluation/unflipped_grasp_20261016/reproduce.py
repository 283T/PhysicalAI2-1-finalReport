"""Offline pickup/lift/nest lookahead using saved images and robot-only feedback."""

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
from evidence_io import write_json_atomic
from fixture_collision_bounds import FixtureBounds
from gripper_model import measured_rest_gap
from plan_held_wire_transfer import plan_from_feedback
from plan_reach import load_arms
from plan_wire_pickup import plan_wire_pickup
from wire_preshape import closing_support_bounds

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
if args.output.exists():
    raise ValueError("Use a new study directory")
args.output.mkdir(parents=True)
source = Path("docs/evaluation/cooperative_nest_20261016/centred_unplanned_connector_contact")
world, models = source / "case/world.sdf", source / "models"
history_path = source / "pickup/trajectory.json.gz"
history = json.loads(gzip.decompress(history_path.read_bytes()))
measured = next(row for row in reversed(history) if row["phase"] == "support_preshape")
# Deliberately discard every object evaluation pose; no native object target.
row = {"sim_time_s": measured["sim_time_s"], "arms": measured["arms"]}
observation_path = source / "pickup/suspended_wire_observation/wire_supports.json"
observation = json.loads(observation_path.read_text())
relation_path = source / "pickup/nest_plan_request.json"
relation = np.array(json.loads(relation_path.read_text())["vision_connector_from_tool"])
arms = load_arms(world, models)
bounds = {name: ArmCollisionBounds(models / (name + ".urdf"), arm) for name, arm in arms.items()}
closing, _ = closing_support_bounds(bounds, row, "left_arm")
current = {name: np.array(state["q_rad"]) for name, state in row["arms"].items()}
full_bounds = {
    name: FingerStateBounds(
        bound,
        {
            name + "_finger_" + side + "_joint": interval.tolist()
            for side, interval in zip(
                ("left", "right"),
                finger_collision_intervals(row["arms"][name]["fingers"], full_stroke=True),
            )
        },
    )
    for name, bound in bounds.items()
}
fixtures = FixtureBounds(world, include_carrier=True)
summary = {
    "scope": "Offline geometry only. Alternate pickup and lift endpoints are NOMINAL assumptions, not measured holding or fresh state. Saved RGB-D relation is assumed unchanged; execution must reacquire images and replan. No physical collision, seating, insertion or acceptance success claim.",
    "physical_tested": False,
    "final_acceptance_eligible": False,
    "source_robot_time_s": row["sim_time_s"],
    "attempts": [],
    "selected": None,
}
inputs = [world, history_path, observation_path, relation_path, *sorted(models.glob("*.urdf"))]
summary["input_sha256"] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
for face, half in (
    (2, True),
    (0, False),
    (0, True),
    (1, False),
    (1, True),
    (3, False),
    (3, True),
    (2, False),
):
    index = len(summary["attempts"])
    record = {"face_quarter_turns": face, "half_turn": half, "geometry_success": False}
    summary["attempts"].append(record)
    try:
        pickup = plan_wire_pickup(
            arms,
            closing,
            world,
            observation,
            current,
            "left_arm",
            "P3",
            include_lift=False,
            include_carrier=True,
            tip_depth_m=0.019,
            wire_profile=models / "left_arm.urdf",
            face_quarter_turns=face,
            half_turn=half,
            route_transit=True,
        )
        write_json_atomic(args.output / f"candidate_{index}_pickup.json", pickup)
        if not pickup["success"]:
            raise ValueError("Pickup fixture screen rejected")
        closed = {name: q.copy() for name, q in current.items()}
        closed["left_arm"] = np.array(pickup["paths"]["approach"]["left_arm"][-1])
        _, lift = shared_lift(arms, closed)
        record["lift_screens"] = {
            "arms": screen_paths(full_bounds, lift),
            "fixtures": fixtures.screen(full_bounds, lift, margin=0.001),
        }
        if not all(v["clear"] for v in record["lift_screens"].values()):
            raise ValueError("Nominal shared lift collision screen rejected")
        nominal_fingers = (0.008 - measured_rest_gap(models / "left_arm.urdf", "left_arm_")) / 2
        nominal = {
            "sim_time_s": row["sim_time_s"],
            "arms": {
                name: {
                    "q_rad": path[-1].tolist(),
                    "fingers": [nominal_fingers] * 2
                    if name == "left_arm"
                    else row["arms"][name]["fingers"],
                }
                for name, path in lift.items()
            },
        }
        nest = plan_from_feedback(
            world,
            arms,
            bounds,
            nominal,
            relation,
            full_finger_stroke=True,
            rigid_transfer=True,
            nest_orientation="unflipped",
        )
        write_json_atomic(args.output / f"candidate_{index}_nest.json", nest)
        record.update(
            nominal_lift_state=nominal,
            geometry_success=nest["success"],
            nest_attempts=len(nest["attempts"]),
        )
        if nest["success"]:
            summary["selected"] = index
    except ValueError as error:
        record["reason"] = str(error)
        record["diagnostics"] = getattr(error, "diagnostics", None)
    write_json_atomic(args.output / "summary.json", summary)
    print(
        json.dumps(
            {
                "candidate": index,
                **{k: record[k] for k in ("face_quarter_turns", "half_turn", "geometry_success")},
                "reason": record.get("reason"),
            }
        ),
        flush=True,
    )
    if summary["selected"] is not None:
        break
