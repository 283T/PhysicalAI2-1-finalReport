"""Park one released arm using robot feedback and fixed geometry only."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from arm_collision_bounds import (
    ArmCollisionBounds,
    FingerStateBounds,
    finger_collision_intervals,
    first_overlap,
    screen_paths,
    with_parked_arms,
)
from evidence_io import write_json_atomic
from fixture_collision_bounds import FixtureBounds
from joint_route import joint_route
from plan_reach import load_arms


def parking_plan(world, arms, bounds, feedback, moving):
    if (
        set(feedback) != {"sim_time_s", "arms"}
        or set(feedback["arms"]) != set(arms)
        or set(bounds) != set(arms)
        or moving not in arms
        or len(arms) != 2
    ):
        raise ValueError("Both robot-only feedback states required")
    if not np.isfinite(feedback["sim_time_s"]) or feedback["sim_time_s"] < 0:
        raise ValueError("Valid robot feedback time required")
    current = {}
    full = {}
    allowed = {
        "q_rad",
        "fingers",
        "tcp",
        "tcp_rotation_world",
        "fk_native_error_m",
        "command_q_rad",
        "command_tcp_pose",
    }
    for name, arm in arms.items():
        state = feedback["arms"][name]
        if not set(state) <= allowed:
            raise ValueError("Object state must not enter robot-only parking planning")
        q = np.asarray(state["q_rad"], dtype=float)
        if q.shape != (len(arm.names),) or not np.isfinite(q).all():
            raise ValueError("Finite joint feedback required")
        fingers = np.asarray(state["fingers"], dtype=float)
        finger_collision_intervals(fingers)
        if np.min(fingers) < 0.0175:
            raise ValueError("Both arms must have opened before parking")
        current[name] = q
        bound = bounds[name]
        while isinstance(bound, FingerStateBounds):
            bound = bound.bounds
        strokes = {
            j.get("name"): [-0.0005, 0.0205] for j in bound.joints if j.get("type") == "prismatic"
        }
        if len(strokes) != 2:
            raise ValueError("Known two-finger mechanics required")
        full[name] = FingerStateBounds(bound, strokes)
    fixtures = FixtureBounds(world, include_carrier=True)
    parked = {n: q for n, q in current.items() if n != moving}

    def valid(q):
        state = {moving: q, **parked}
        return (
            first_overlap(full, state) is None
            and fixtures.first_overlap(full, state, margin=0.001) is None
        )

    route = joint_route(
        current[moving],
        np.zeros(len(arms[moving].names)),
        arms[moving].limits,
        valid,
        max_nodes=600,
    )
    result = {
        "success": False,
        "moving_arm": moving,
        "starting_sim_time_s": feedback["sim_time_s"],
        "starting_joints_rad": {n: q.tolist() for n, q in current.items()},
        "route": route,
        "scope": "Robot-only static parking check after physical release. Full finger strokes; no cable collision or passive-seat stability proof",
    }
    if not route["success"]:
        return result
    paths = {moving: np.asarray(route["path"])}
    combined = with_parked_arms(paths, parked)
    result["checks"] = {
        "arms": screen_paths(full, combined),
        "fixtures": fixtures.screen(full, combined, margin=0.001),
    }
    result["success"] = all(c["clear"] for c in result["checks"].values())
    if result["success"]:
        result["paths"] = {n: q.tolist() for n, q in paths.items()}
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("world", "models", "request", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("Use a new output file")
    request = json.loads(args.request.read_text())
    arms = load_arms(args.world, args.models)
    bounds = {n: ArmCollisionBounds(args.models / (n + ".urdf"), a) for n, a in arms.items()}
    result = parking_plan(
        args.world, arms, bounds, request["robot_feedback"], request["moving_arm"]
    )
    result.update(
        request_sha256=hashlib.sha256(args.request.read_bytes()).hexdigest(),
        world_sha256=hashlib.sha256(args.world.read_bytes()).hexdigest(),
        model_sha256={
            n: hashlib.sha256((args.models / (n + ".urdf")).read_bytes()).hexdigest() for n in arms
        },
    )
    write_json_atomic(args.output, result)
    if not result["success"]:
        raise SystemExit(1)
