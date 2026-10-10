"""Offline route study from a successful pickup; never commands the simulator."""

import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path

import numpy as np
from arm_collision_bounds import ArmCollisionBounds, screen_paths
from assembly_case import validate_connection_plan
from cooperative_transfer import assess_pair_path, paired_path, socket_poses
from fixture_collision_bounds import FixtureBounds
from plan_reach import load_arms


def study(source, models):
    validation = json.loads((source / "validation.json").read_text())
    world = source / "world.sdf"
    if (
        not validation["success"]
        or validation["world_sha256"] != hashlib.sha256(world.read_bytes()).hexdigest()
    ):
        raise ValueError("A successful pickup with its matching world is required")
    board = json.loads((source / "board.json").read_text())
    connection = json.loads((source / "connection_plan.json").read_text())
    rows = validate_connection_plan(connection, board)
    assignment = validation["assignment"]
    if len(assignment) != 2:
        raise ValueError("Two-terminal pickup required")
    primary = next(n for n, plug in assignment.items() if plug == validation["plug_id"])
    row = next(row for row in rows if row["plug_id"] == validation["plug_id"])
    grasp = json.loads((source / "grasp_plan.json").read_text())
    candidate = next(r for r in grasp["connectors"] if r["plug_id"] == row["plug_id"])[
        "grasp_candidate"
    ]
    arms = load_arms(world, models)
    bounds = {
        name: ArmCollisionBounds(models / (name + ".urdf"), arm) for name, arm in arms.items()
    }
    fixtures = FixtureBounds(world)
    current = {
        name: np.asarray(path[-1])
        for name, path in validation["prepared_joint_paths"]["lift"].items()
    }
    targets = socket_poses(board, row, candidate["connector_from_tool"])
    report = {
        "success": False,
        "stage": "offline_board_hover",
        "assignment": assignment,
        "primary": primary,
        "socket": row["socket"],
        "scope": "Offline nominal lift-end states, inter-arm and static fixture box screening; no physical transfer/insertion, self/cable/carrier collision or tension validation",
        "world_sha256": validation["world_sha256"],
        "socket_tool_poses": {name: pose.tolist() for name, pose in targets.items()},
        "attempts": [],
        "input_sha256": {
            name: hashlib.sha256((source / name).read_bytes()).hexdigest()
            for name in ("validation.json", "board.json", "connection_plan.json", "grasp_plan.json")
        },
        "model_sha256": {
            name: hashlib.sha256((models / (name + ".urdf")).read_bytes()).hexdigest()
            for name in arms
        },
    }
    # Compare alternative rotation locations before changing pickup layout or gains.
    start = arms[primary].forward(current[primary])
    for orbit, height, order in itertools.product(
        (None, 0.0, -math.pi / 2, math.pi / 2),
        (0.30, 0.40, 0.50),
        ("translate_then_rotate", "rotate_then_translate", "combined"),
    ):
        raised = start.copy()
        raised[2, 3] = max(start[2, 3], height)
        elevated_goal = targets["hover"].copy()
        elevated_goal[2, 3] = height
        intermediate = raised.copy()
        if order == "translate_then_rotate":
            intermediate[:3, 3] = elevated_goal[:3, 3]
        else:
            intermediate[:3, :3] = elevated_goal[:3, :3]
        goals = (
            [raised]
            + ([] if order == "combined" else [intermediate])
            + [elevated_goal, targets["hover"]]
        )
        attempt = {
            "height_m": height,
            "support_orbit_rad": orbit,
            "order": order,
            "phases": [],
            "success": False,
        }
        state = current.copy()
        for index, goal in enumerate(goals):
            phase = {"index": index}
            attempt["phases"].append(phase)
            try:
                phase_orbit = None if orbit is None else (orbit if index == 1 else 0.0)
                paths = paired_path(arms, state, primary, goal, support_orbit=phase_orbit)
                phase["pair"] = assess_pair_path(arms, paths, rigid=orbit is None)
                phase["collision"] = screen_paths(bounds, paths)
                phase["fixtures"] = fixtures.screen(bounds, paths)
                if (
                    not phase["pair"]["success"]
                    or not phase["collision"]["clear"]
                    or not phase["fixtures"]["clear"]
                ):
                    phase["reason"] = "Pair constraint, inter-arm or fixture box screen rejected"
                    break
                phase["joint_paths"] = {name: path.tolist() for name, path in paths.items()}
                state = {name: path[-1] for name, path in paths.items()}
            except ValueError as error:
                phase["reason"] = str(error)
                break
        else:
            attempt["success"] = True
            report["success"] = True
        report["attempts"].append(attempt)
        if attempt["success"]:
            # Descent is diagnostic only: never authorize contact insertion here.
            attempt["descent"] = {"success": False}
            try:
                paths = paired_path(
                    arms,
                    state,
                    primary,
                    targets["seated"],
                    support_orbit=0.0 if orbit is not None else None,
                )
                descent = attempt["descent"]
                descent["pair"] = assess_pair_path(arms, paths, rigid=orbit is None)
                descent["collision"] = screen_paths(bounds, paths)
                descent["fixtures"] = fixtures.screen(bounds, paths)
                descent["success"] = (
                    descent["pair"]["success"]
                    and descent["collision"]["clear"]
                    and descent["fixtures"]["clear"]
                )
                if descent["success"]:
                    descent["joint_paths"] = {name: path.tolist() for name, path in paths.items()}
            except ValueError as error:
                attempt["descent"]["reason"] = str(error)
            if "selected_attempt" not in report or attempt["descent"]["success"]:
                report["selected_attempt"] = len(report["attempts"]) - 1
            if attempt["descent"]["success"]:
                break
    # Keep executable-sized geometry data only for the selected offline route.
    for index, attempt in enumerate(report["attempts"]):
        if index != report.get("selected_attempt"):
            for phase in attempt["phases"]:
                phase.pop("joint_paths", None)
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--models", type=Path, default=Path("outputs/melfa_worlds"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--require-descent",
        action="store_true",
        help="Return failure if no collision-screened descent was found",
    )
    args = parser.parse_args()
    args.output.unlink(missing_ok=True)
    report = study(args.source, args.models)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    selected = report["attempts"][report["selected_attempt"]] if report["success"] else None
    descent_success = bool(selected and selected["descent"]["success"])
    print(
        json.dumps(
            {
                "hover_success": report["success"],
                "descent_success": descent_success,
                "attempts": len(report["attempts"]),
            }
        )
    )
    if not report["success"] or (args.require_descent and not descent_success):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
