"""Continuous paired transfer to a passive nest, retaining the existing side grip."""

import argparse
import hashlib
import itertools
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from arm_collision_bounds import ArmCollisionBounds, screen_paths, with_parked_arms
from arm_kinematics import transform
from assembly_case import validate_connection_plan
from cooperative_transfer import assess_pair_path, paired_path, rigid_pose, single_arm_path
from fixture_collision_bounds import FixtureBounds, sdf_pose
from plan_reach import load_arms


def nest_connector_pose(world):
    """Derive the seat from the actual SDF support faces, not a second coordinate copy."""
    fixture = ET.parse(world).getroot().find("world/model[@name='fixtures']")
    if fixture is None or fixture.findtext("static") != "true":
        raise ValueError("Static factory fixtures required")
    supports = []
    for link in fixture.findall("link"):
        for collision in link.findall("collision"):
            if not collision.get("name", "").startswith("regrasp_end_support_"):
                continue
            pose = sdf_pose(fixture) @ sdf_pose(link) @ sdf_pose(collision)
            size = np.asarray([float(v) for v in collision.findtext("geometry/box/size").split()])
            if size.shape != (3,) or not np.isfinite(size).all() or min(size) <= 0:
                raise ValueError("Invalid nest support dimensions")
            if not np.allclose(pose[:3, 2], [0, 0, 1]):
                raise ValueError("Nest supports must be horizontal")
            supports.append((pose, pose[2, 3] + size[2] / 2))
    if len(supports) not in (2, 4) or np.ptp([top for _, top in supports]) > 1e-8:
        raise ValueError("A complete, level two-ended nest is required")
    rotation = supports[0][0][:3, :3]
    if any(not np.allclose(pose[:3, :3], rotation) for pose, _ in supports):
        raise ValueError("Nest support rotations disagree")
    pose = np.eye(4)
    pose[:3, :3] = rotation
    pose[:2, 3] = np.mean([p[:2, 3] for p, _ in supports], axis=0)
    pose[2, 3] = supports[0][1] + 0.01
    return pose


def plan_to_nest(
    arms,
    bounds,
    world,
    current,
    primary,
    relation,
    support_rises=(0.0, 0.1, 0.15),
    *,
    parked=None,
    include_carrier=False,
):
    """Plan from supplied current joints; never replace them with a new IK branch."""
    relation = rigid_pose(relation)
    if primary not in arms or set(current) != set(arms) or len(arms) not in (1, 2):
        raise ValueError("One or two active arms with matching current joints required")
    single = len(arms) == 1
    parked = {} if parked is None else parked
    initial_paths = with_parked_arms(
        {name: np.repeat(np.asarray(q)[None, :], 2, axis=0) for name, q in current.items()}, parked
    )
    if len(bounds) != 2 or set(initial_paths) != set(bounds):
        raise ValueError("Every arm must have a current moving or parked joint observation")
    if not support_rises or not np.isfinite(support_rises).all() or min(support_rises) < 0:
        raise ValueError("Finite nonnegative support rises required")
    seat = nest_connector_pose(world)
    fixtures = FixtureBounds(world, include_carrier=include_carrier)
    start = arms[primary].forward(current[primary])
    report = {
        "success": False,
        "includes_stopped_carrier": include_carrier,
        "mode": "single_with_parked_arm" if single else "paired",
        "parked_joints_rad": {name: np.asarray(q).tolist() for name, q in parked.items()},
        "nest_connector_pose": seat.tolist(),
        "attempts": [],
        "scope": "Continuous IK from the supplied lift state; inter-arm and static fixture box screening only. No physical placement, self/cable/object collision or force validation",
    }
    for flipped, height, orbit, order, rise in itertools.product(
        (False, True) if single else (True, False),
        (0.35, 0.45),
        (None,) if single else (0.0, -math.pi / 2, math.pi / 2, None),
        ("translate_then_rotate", "rotate_then_translate"),
        (0.0,) if single else support_rises,
    ):
        if orbit is None and rise != 0:
            continue
        connector = seat @ transform(rpy=(0, 0, math.pi if flipped else 0))
        goal = connector @ relation
        hover = goal.copy()
        hover[2, 3] += 0.10
        raised = start.copy()
        raised[2, 3] = max(height, start[2, 3])
        elevated = goal.copy()
        elevated[2, 3] = raised[2, 3]
        intermediate = raised.copy()
        if order == "translate_then_rotate":
            intermediate[:3, 3] = elevated[:3, 3]
        else:
            intermediate[:3, :3] = elevated[:3, :3]
        attempt = {
            "flipped": flipped,
            "height_m": height,
            "orbit_rad": orbit,
            "order": order,
            "support_rise_m": rise,
            "success": False,
            "phases": [],
        }
        state = {name: np.asarray(q).copy() for name, q in current.items()}
        for index, target in enumerate((raised, intermediate, elevated, hover, goal)):
            phase = {"index": index, "target": target.tolist()}
            attempt["phases"].append(phase)
            try:
                rotation_phase = 2 if order == "translate_then_rotate" else 1
                if single:
                    paths = {primary: single_arm_path(arms[primary], state[primary], target)}
                else:
                    paths = paired_path(
                        arms,
                        state,
                        primary,
                        target,
                        support_orbit=None
                        if orbit is None
                        else orbit
                        if index == rotation_phase
                        else 0.0,
                        support_rise=rise if index == 4 else 0.0,
                    )
                    phase["pair"] = assess_pair_path(arms, paths, rigid=orbit is None)
                screened = with_parked_arms(paths, parked)
                phase["collision"] = screen_paths(bounds, screened)
                phase["fixtures"] = fixtures.screen(bounds, screened, margin=0.001)
                if (
                    not phase.get("pair", {"success": True})["success"]
                    or not phase["collision"]["clear"]
                    or not phase["fixtures"]["clear"]
                ):
                    phase["reason"] = "Kinematics or collision screening rejected"
                    break
                phase["joint_paths"] = {name: path.tolist() for name, path in paths.items()}
                state = {name: path[-1] for name, path in paths.items()}
            except ValueError as error:
                phase["reason"] = str(error)
                break
        else:
            attempt["success"] = True
            report.update(
                success=True,
                selected_attempt=len(report["attempts"]),
                goal_connector_pose=connector.tolist(),
                connector_from_tool=relation.tolist(),
            )
        report["attempts"].append(attempt)
        if attempt["success"]:
            break
        for phase in attempt["phases"]:
            phase.pop("joint_paths", None)
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--models", type=Path, required=True)
    parser.add_argument("--source-models", type=Path, default=Path("outputs/melfa_worlds"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--support-rises", type=float, nargs="+", default=[0, 0.1, 0.15])
    args = parser.parse_args()
    args.output.unlink(missing_ok=True)
    validation = json.loads((args.source / "validation.json").read_text())
    source_world = args.source / "world.sdf"
    if (
        not validation["success"]
        or validation["world_sha256"] != hashlib.sha256(source_world.read_bytes()).hexdigest()
        or len(validation["assignment"]) != 2
    ):
        raise ValueError("Matching successful paired pickup required")
    if json.loads((args.source / "board.json").read_text()) != json.loads(
        (args.world.parent / "board.json").read_text()
    ):
        raise ValueError("Target world changed the matched board")
    board = json.loads((args.source / "board.json").read_text())
    connection = json.loads((args.source / "connection_plan.json").read_text())
    validate_connection_plan(connection, board)
    if (
        connection.get("image_sha256")
        != hashlib.sha256((args.world.parent / "drawing.png").read_bytes()).hexdigest()
    ):
        raise ValueError("Target drawing does not match the locally inferred connection plan")
    primary = next(
        name for name, plug in validation["assignment"].items() if plug == validation["plug_id"]
    )
    grasp = json.loads((args.source / "grasp_plan.json").read_text())
    relation = next(row for row in grasp["connectors"] if row["plug_id"] == validation["plug_id"])[
        "grasp_candidate"
    ]["connector_from_tool"]
    arms = load_arms(args.world, args.models)
    old_arms = load_arms(source_world, args.source_models)
    current = {
        name: np.asarray(path[-1])
        for name, path in validation["prepared_joint_paths"]["lift"].items()
    }
    for name in arms:
        for q in (current[name], np.zeros(len(arms[name].names))):
            if not np.allclose(arms[name].forward(q), old_arms[name].forward(q), atol=1e-10):
                raise ValueError("Target world changed the arm kinematics")
    bounds = {
        name: ArmCollisionBounds(args.models / (name + ".urdf"), arm) for name, arm in arms.items()
    }
    report = plan_to_nest(arms, bounds, args.world, current, primary, relation, args.support_rises)
    report.update(
        primary=primary,
        assignment=validation["assignment"],
        start_joints={n: q.tolist() for n, q in current.items()},
        world_sha256=hashlib.sha256(args.world.read_bytes()).hexdigest(),
        model_sha256={
            name: hashlib.sha256((args.models / (name + ".urdf")).read_bytes()).hexdigest()
            for name in arms
        },
        input_sha256={
            name: hashlib.sha256((args.source / name).read_bytes()).hexdigest()
            for name in (
                "world.sdf",
                "validation.json",
                "board.json",
                "grasp_plan.json",
                "connection_plan.json",
            )
        },
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"success": report["success"], "attempts": len(report["attempts"])}))
    if not report["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
