"""Isolated top-regrasp planner using fresh RGB-D and robot feedback only."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from arm_collision_bounds import ArmCollisionBounds, FingerStateBounds, finger_collision_intervals
from cooperative_transfer import rigid_pose
from evidence_io import write_json_atomic
from plan_reach import load_arms
from regrasp_plan import plan_top_pickup


def full_stroke_bounds(bounds):
    result = {}
    for name, raw in bounds.items():
        while isinstance(raw, FingerStateBounds):
            raw = raw.bounds
        intervals = {
            j.get("name"): [-0.0005, 0.0205] for j in raw.joints if j.get("type") == "prismatic"
        }
        if len(intervals) != 2:
            raise ValueError("Known two-finger mechanical stroke required")
        result[name] = FingerStateBounds(raw, intervals)
    return result


def plan_request(world, arms, bounds, request):
    if set(request) != {"robot_feedback", "vision_connector_pose", "primary_arm", "pose_source"}:
        raise ValueError("Explicit vision pose and robot-only request required")
    if request["pose_source"] != "fresh_rgbd":
        raise ValueError("Fresh RGB-D pose required")
    current = robot_joints(request["robot_feedback"], arms, require_open=True)
    body = rigid_pose(request["vision_connector_pose"])
    return plan_top_pickup(
        arms, full_stroke_bounds(bounds), world, current, request["primary_arm"], body
    )


def robot_joints(feedback, arms, *, require_open):
    if set(feedback) != {"sim_time_s", "arms"} or set(feedback["arms"]) != set(arms):
        raise ValueError("Both robot states required; no object evaluation feedback")
    if not np.isfinite(feedback["sim_time_s"]) or feedback["sim_time_s"] < 0:
        raise ValueError("Finite robot feedback timestamp required")
    current = {}
    for name, arm in arms.items():
        state = feedback["arms"][name]
        allowed = {
            "q_rad",
            "fingers",
            "tcp",
            "tcp_rotation_world",
            "fk_native_error_m",
            "command_q_rad",
            "command_tcp_pose",
        }
        if not set(state) <= allowed:
            raise ValueError("Object evaluation state cannot enter the planner")
        q = np.asarray(state["q_rad"], dtype=float)
        if q.shape != (len(arm.names),) or not np.isfinite(q).all():
            raise ValueError("Finite joint feedback required")
        fingers = np.asarray(state["fingers"], dtype=float)
        finger_collision_intervals(fingers)
        if require_open and min(fingers) < 0.0175:
            raise ValueError("Open arms required for top-regrasp planning")
        current[name] = q
    return current


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("world", "models", "request", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("New plan output required")
    request = json.loads(args.request.read_text())
    arms = load_arms(args.world, args.models)
    bounds = {n: ArmCollisionBounds(args.models / (n + ".urdf"), a) for n, a in arms.items()}
    result = plan_request(args.world, arms, bounds, request)
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


if __name__ == "__main__":
    main()
