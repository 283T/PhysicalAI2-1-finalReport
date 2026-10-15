"""Isolated loaded insertion planner; no evaluation object pose inputs."""

import argparse
import hashlib
import json
from pathlib import Path

from arm_collision_bounds import ArmCollisionBounds
from cooperative_transfer import rigid_pose
from evidence_io import write_json_atomic
from plan_reach import load_arms
from top_pickup_plan import robot_joints
from top_socket_plan import plan_socket


def plan_request(world, arms, bounds, board, drawing_plan, request):
    if (
        set(request)
        != {"robot_feedback", "vision_connector_from_tool", "primary_arm", "plug_id", "pose_source"}
        or request["pose_source"] != "fresh_rgbd"
    ):
        raise ValueError("Explicit fresh RGB-D relation and robot-only feedback required")
    current = robot_joints(request["robot_feedback"], arms, require_open=False)
    relation = rigid_pose(request["vision_connector_from_tool"])
    return plan_socket(
        arms,
        bounds,
        world,
        current,
        request["primary_arm"],
        board,
        drawing_plan,
        request["plug_id"],
        relation,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("world", "models", "request", "drawing-plan", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("Preserve previous insertion plans")
    board_path = args.world.parent / "board.json"
    arms = load_arms(args.world, args.models)
    bounds = {n: ArmCollisionBounds(args.models / (n + ".urdf"), a) for n, a in arms.items()}
    result = plan_request(
        args.world,
        arms,
        bounds,
        json.loads(board_path.read_text()),
        json.loads(args.drawing_plan.read_text()),
        json.loads(args.request.read_text()),
    )
    result.update(
        request_sha256=hashlib.sha256(args.request.read_bytes()).hexdigest(),
        world_sha256=hashlib.sha256(args.world.read_bytes()).hexdigest(),
        model_sha256={
            n: hashlib.sha256((args.models / (n + ".urdf")).read_bytes()).hexdigest() for n in arms
        },
        board_sha256=hashlib.sha256(board_path.read_bytes()).hexdigest(),
        drawing_plan_sha256=hashlib.sha256(args.drawing_plan.read_bytes()).hexdigest(),
    )
    write_json_atomic(args.output, result)
    if not result["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
