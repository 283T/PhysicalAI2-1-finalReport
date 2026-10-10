"""Insertion geometry after an explicit, not-yet-executed top-face regrasp."""

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from arm_collision_bounds import ArmCollisionBounds, screen_paths
from assembly_case import validate_connection_plan
from cooperative_transfer import assess_pair_path, paired_path, socket_poses
from fixture_collision_bounds import FixtureBounds
from plan_reach import load_arms


def top_grasp_relation(flipped=False):
    """Close tool-X fingers along connector-Y; approach along connector minus-Z."""
    pose = np.eye(4)
    pose[:3, :3] = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
    if flipped:
        pose[:3, :2] *= -1
    pose[2, 3] = -0.006
    return pose


def evaluate_descent(arms, bounds, fixtures, current, primary, board, row):
    """Select a continuous top-grip descent; starts require a physical regrasp."""
    report = {"success": False, "attempts": []}
    for flipped in (False, True):
        relation = top_grasp_relation(flipped)
        targets = socket_poses(board, row, relation)
        arm = arms[primary]
        direction = targets["hover"][:2, 3] - arm.base[:2, 3]
        waist = math.atan2(direction[1], direction[0]) - math.atan2(arm.base[1, 0], arm.base[0, 0])
        waist = (waist + math.pi) % (2 * math.pi) - math.pi
        seeds = [
            np.asarray(current[primary]),
            np.array([waist, 1.2, 1, 0, 1, 0]),
            np.array([waist, 1.2, 1, math.pi, -1, 0]),
            np.array([waist, 1.4, 0.8, 0, 1.2, 0]),
        ]
        seeds += list(
            np.random.default_rng(0).uniform(arm.limits[:, 0] * 0.7, arm.limits[:, 1] * 0.7, (8, 6))
        )
        for seed_index, seed in enumerate(seeds):
            attempt = {"flipped": flipped, "seed_index": seed_index, "success": False}
            report["attempts"].append(attempt)
            hover = arm.inverse(targets["hover"], seed=seed, attempts=1)
            if not hover["success"]:
                attempt["reason"] = "Top-grip hover IK failed"
                continue
            start = {name: np.asarray(q).copy() for name, q in current.items()}
            start[primary] = np.asarray(hover["q_rad"])
            try:
                paths = paired_path(arms, start, primary, targets["seated"], support_orbit=0.0)
                attempt["pair"] = assess_pair_path(arms, paths, rigid=False)
                attempt["collision"] = screen_paths(bounds, paths)
                attempt["fixtures"] = fixtures.screen(bounds, paths, margin=0.001)
                attempt["success"] = (
                    attempt["pair"]["success"]
                    and attempt["collision"]["clear"]
                    and attempt["fixtures"]["clear"]
                )
                if attempt["success"]:
                    report.update(
                        success=True,
                        selected_attempt=len(report["attempts"]) - 1,
                        connector_from_tool=relation.tolist(),
                        tool_poses={name: pose.tolist() for name, pose in targets.items()},
                        joint_paths={name: path.tolist() for name, path in paths.items()},
                    )
                    return report
            except ValueError as error:
                attempt["reason"] = str(error)
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--transfer-report", type=Path, required=True)
    parser.add_argument("--models", type=Path, default=Path("outputs/melfa_worlds"))
    parser.add_argument(
        "--target-world",
        type=Path,
        help="Check the post-regrasp fixture scene instead of the original pickup scene",
    )
    parser.add_argument("--target-models", type=Path, help="URDF directory for --target-world")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if bool(args.target_world) != bool(args.target_models):
        parser.error("--target-world and --target-models must be provided together")
    args.output.unlink(missing_ok=True)
    transfer = json.loads(args.transfer_report.read_text())
    world = args.source / "world.sdf"
    if (
        not transfer["success"]
        or transfer["world_sha256"] != hashlib.sha256(world.read_bytes()).hexdigest()
    ):
        raise ValueError("A matching successful offline hover route is required")
    for name, digest in transfer["input_sha256"].items():
        if hashlib.sha256((args.source / name).read_bytes()).hexdigest() != digest:
            raise ValueError("Source changed since the transfer study: " + name)
    arms = load_arms(world, args.models)
    if transfer["model_sha256"] != {
        name: hashlib.sha256((args.models / (name + ".urdf")).read_bytes()).hexdigest()
        for name in arms
    }:
        raise ValueError("Models changed since the transfer study")
    board = json.loads((args.source / "board.json").read_text())
    plan = json.loads((args.source / "connection_plan.json").read_text())
    rows = validate_connection_plan(plan, board)
    row = next(r for r in rows if r["plug_id"] == transfer["assignment"][transfer["primary"]])
    final = transfer["attempts"][transfer["selected_attempt"]]["phases"][-1]["joint_paths"]
    current = {name: np.asarray(path[-1]) for name, path in final.items()}
    if args.target_world:
        target_board = json.loads((args.target_world.parent / "board.json").read_text())
        if target_board != board:
            raise ValueError("Target scene board differs from the validated drawing")
        target_arms = load_arms(args.target_world, args.target_models)
        for name in arms:
            for q in (current[name], np.zeros(len(arms[name].names))):
                if not np.allclose(arms[name].forward(q), target_arms[name].forward(q), atol=1e-10):
                    raise ValueError("Target scene changed arm bases or kinematics")
        arms = target_arms
        world = args.target_world
    active_models = args.target_models or args.models
    bounds = {
        name: ArmCollisionBounds(active_models / (name + ".urdf"), arm)
        for name, arm in arms.items()
    }
    report = evaluate_descent(
        arms, bounds, FixtureBounds(world), current, transfer["primary"], board, row
    )
    report.update(
        stage="offline_post_regrasp_descent",
        primary=transfer["primary"],
        plug_id=row["plug_id"],
        socket=row["socket"],
        scope="Assumes a top-face regrasp at hover, not an executable transition from the old grip; no physical insertion, self/cable/contact/force validation",
        world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
        model_sha256={
            name: hashlib.sha256((active_models / (name + ".urdf")).read_bytes()).hexdigest()
            for name in arms
        },
        source_world_sha256=transfer["world_sha256"],
        transfer_sha256=hashlib.sha256(args.transfer_report.read_bytes()).hexdigest(),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"success": report["success"], "attempts": len(report["attempts"])}))
    if not report["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
