"""Connector first, then suspended-wire support, owned by one ROS arm session."""

import argparse
import gzip
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np
import rclpy
from arm_kinematics import rotation_error
from arm_observation import acquire_observation
from arm_session import ArmSession
from assembly_case import validate_connection_plan
from cooperative_pick import prepare_pickup_paths, shared_lift
from cooperative_transfer import rigid_pose
from evidence_io import write_json_atomic
from grasp_validation import (
    assess_hold,
    assess_opening,
    assess_preclose_pose,
)
from grip_guard import register_grip_guard
from gripper_model import measured_rest_gap
from held_wire_nest import place_in_nest
from pick_wire import evaluation_state
from plan_wire_pickup import plan_wire_pickup, wire_grasp_target
from preclose_alignment import correction_budget, correction_path, tracking_compensated_goal


def primary_pickup(session, output, calibration, report, checkpoint, guard_stream):
    name = "right_arm"
    checkpoint("primary_observation")
    initial = acquire_observation(session, output / "primary_observation")
    pose = initial["connectors"][0]["pose_6d"]
    body = np.eye(4)
    body[:3, :3], body[:3, 3] = pose["rotation_world_from_connector"], pose["center_xyz_m"]
    grip = rigid_pose(pose["grasp_tcp_pose"])
    relation = np.linalg.inv(rigid_pose(body)) @ grip
    hover = grip.copy()
    hover[:3, 3] -= grip[:3, 2] * 0.10
    row = session.snapshot()
    current = {n: np.array(s["q_rad"]) for n, s in row["arms"].items()}
    paths, selected = prepare_pickup_paths(
        {name: session.arms[name]},
        {name: current[name]},
        {name: hover},
        {name: grip},
        session.bounds,
        parked={"left_arm": current["left_arm"]},
        precise_endpoints=True,
    )
    report["primary_branch_selection"] = selected
    for phase in ("hover", "approach"):
        checkpoint("primary_" + phase)
        session.move(
            "primary_" + phase,
            paths[phase],
            cartesian_speed_m_s=0.02 if phase == "approach" else None,
        )
    budget = {}
    for attempt in range(3):
        checkpoint("primary_preclose_" + str(attempt))
        observation = acquire_observation(
            session, output / ("primary_preclose_" + str(attempt)), calibration=calibration
        )
        detected = observation["connectors"][0]["pose_6d"]
        actual = session.snapshot()["arms"][name]
        tool = np.eye(4)
        tool[:3, :3], tool[:3, 3] = actual["tcp_rotation_world"], actual["tcp"]
        check = assess_preclose_pose(detected, tool, relation)
        check["preferred_alignment"] = (
            check["position_error_m"] < 0.0005 and check["angle_error_rad"] < 0.005
        )
        report.setdefault("primary_preclose_checks", []).append(check)
        if check["success"] and check["preferred_alignment"]:
            break
        if attempt == 2:
            raise RuntimeError("Primary preclose rejected after bounded corrections")
        desired_body = np.eye(4)
        desired_body[:3, :3], desired_body[:3, 3] = (
            detected["rotation_world_from_connector"],
            detected["center_xyz_m"],
        )
        desired = rigid_pose(desired_body) @ relation
        target = tracking_compensated_goal(
            desired, tool, session.arms[name].forward(session.targets[name])
        )
        q = np.array(actual["q_rad"])
        budget = correction_budget(budget, {name: session.arms[name].forward(q)}, {name: target})
        report["primary_correction_budget"] = budget
        session.move("primary_correction", {name: correction_path(session.arms[name], q, target)})
    baseline = evaluation_state(session.snapshot(), name, "P3")
    checkpoint("primary_close")
    session.grips[name] = 0
    closed_row = session.dwell("primary_close", 4)[-1]
    closed = evaluation_state(closed_row, name, "P3")
    rest = report["rest_gaps_m"][name]
    report["primary_closure"] = assess_opening([closed["fingers"]], rest_gap_m=rest)
    if not report["primary_closure"]["success"]:
        raise RuntimeError("Primary closure rejected")
    guard = register_grip_guard(
        session, report, "primary", closed, lambda r: evaluation_state(r, name, "P3"), guard_stream
    )
    _, lifts = shared_lift({name: session.arms[name]}, {name: np.array(closed["q_rad"])})
    checkpoint("primary_lift")
    session.move("primary_lift", lifts, cartesian_speed_m_s=0.02)
    checkpoint("primary_hold")
    held = session.dwell("primary_hold", 4)
    report["primary_hold"] = assess_hold(
        baseline, [evaluation_state(r, name, "P3") for r in held], closed, rest_gap_m=rest
    )
    if not report["primary_hold"]["success"]:
        raise RuntimeError("Primary holding rejected")
    return relation, guard


def run(session, args, report, checkpoint, guard_stream):
    report["rest_gaps_m"] = {
        n: measured_rest_gap(args.models / (n + ".urdf"), n + "_") for n in session.arms
    }
    relation, primary_guard = primary_pickup(
        session, args.output, args.inspection_calibration, report, checkpoint, guard_stream
    )
    report["primary_connector_from_tool"] = relation.tolist()
    checkpoint("suspended_wire_observation")
    observation = acquire_observation(
        session,
        args.output / "suspended_wire_observation",
        wire_distance_mm=args.wire_distance_mm,
        calibration=args.inspection_calibration,
    )
    current = {n: s["q_rad"] for n, s in session.snapshot()["arms"].items()}
    plan = plan_wire_pickup(
        session.arms,
        session.bounds,
        args.world,
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
    write_json_atomic(args.output / "wire_plan.json", plan)
    if not plan["success"]:
        raise RuntimeError("Suspended wire path rejected")
    for phase in ("hover", "approach"):
        checkpoint("support_" + phase)
        session.move(
            "support_" + phase,
            plan["paths"][phase],
            cartesian_speed_m_s=0.02 if phase == "approach" else None,
            settle_time_s=30 if phase == "approach" else 20,
        )
    checkpoint("support_preclose")
    fresh = acquire_observation(
        session,
        args.output / "support_preclose",
        wire_distance_mm=args.wire_distance_mm,
        calibration=args.inspection_calibration,
    )
    before, after = observation["wire_supports"][0], fresh["wire_supports"][0]
    center_error = float(
        np.linalg.norm(np.array(before["wire_center_xyz_m"]) - after["wire_center_xyz_m"])
    )
    tangent_error = float(
        np.arccos(np.clip(abs(np.dot(before["tangent_world"], after["tangent_world"])), 0, 1))
    )
    desired, _ = wire_grasp_target(after, tip_depth_m=0.006, face_quarter_turns=2)
    tool_state = session.snapshot()["arms"]["left_arm"]
    tool_error = float(np.linalg.norm(np.array(tool_state["tcp"]) - desired[:3, 3]))
    tool_angle = float(
        np.linalg.norm(rotation_error(desired[:3, :3], np.array(tool_state["tcp_rotation_world"])))
    )
    report["support_preclose"] = {
        "center_change_m": center_error,
        "tangent_change_rad": tangent_error,
        "tool_position_error_m": tool_error,
        "tool_angle_error_rad": tool_angle,
        "success": center_error < 0.002
        and tangent_error < 0.025
        and tool_error < 0.002
        and tool_angle < 0.025,
    }
    if not report["support_preclose"]["success"]:
        raise RuntimeError("Suspended wire moved before closure")
    row = session.snapshot()
    poses = row["evaluation_poses"]
    # Identity for independent evaluation only, never a goal position.
    from grasp_validation import world_link_positions

    names = [n for n in poses if n.startswith("P3_wire_")]
    points = world_link_positions(poses, "wire_harness_multi", names)
    link = min(names, key=lambda n: np.linalg.norm(points[n] - after["wire_center_xyz_m"]))
    report["support_evaluation_link"] = link
    baselines = {
        "left_arm": evaluation_state(row, "left_arm", link),
        "right_arm": evaluation_state(row, "right_arm", "P3"),
    }
    checkpoint("support_close")
    session.grips["left_arm"] = 0
    closed = session.dwell("support_close", 4)[-1]
    report["support_closure"] = assess_opening(
        [closed["arms"]["left_arm"]["fingers"]], 0.008, rest_gap_m=report["rest_gaps_m"]["left_arm"]
    )
    if not report["support_closure"]["success"]:
        raise RuntimeError("Support closure rejected")
    support_guard = register_grip_guard(
        session,
        report,
        "support",
        evaluation_state(closed, "left_arm", link),
        lambda r: evaluation_state(r, "left_arm", link),
        guard_stream,
    )
    checkpoint("shared_lift")
    _, paths = shared_lift(
        session.arms, {n: np.array(s["q_rad"]) for n, s in closed["arms"].items()}
    )
    session.move("shared_lift", paths, cartesian_speed_m_s=0.02)
    checkpoint("dual_hold")
    held = session.dwell("dual_hold", 4)
    report["dual_holds"] = {
        n: assess_hold(
            baselines[n],
            [evaluation_state(r, n, obj) for r in held],
            evaluation_state(closed, n, obj),
            housing_width=0.008 if n == "left_arm" else 0.020,
            rest_gap_m=report["rest_gaps_m"][n],
        )
        for n, obj in [("left_arm", link), ("right_arm", "P3")]
    }
    report["success"] = all(h["success"] for h in report["dual_holds"].values())
    if not report["success"]:
        raise RuntimeError("Dual holding rejected")
    return {"primary": primary_guard, "support": support_guard}


def main():
    parser = argparse.ArgumentParser()
    for name in ("world", "models", "drawing-plan", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument(
        "--inspection-calibration", type=Path, default=Path("configs/end_inspection_camera.json")
    )
    parser.add_argument("--wire-distance-mm", type=float, default=110)
    parser.add_argument("--place-in-nest", action="store_true")
    args = parser.parse_args()
    if not np.isfinite(args.wire_distance_mm) or not 75 <= args.wire_distance_mm <= 200:
        parser.error("Wire distance must be finite and between 75 and 200 mm")
    case = args.world.parent
    drawing_plan = json.loads(args.drawing_plan.read_text())
    validate_connection_plan(drawing_plan, json.loads((case / "board.json").read_text()))
    if (
        drawing_plan.get("source") != "local_vision_model"
        or drawing_plan.get("image_sha256")
        != hashlib.sha256((case / "drawing.png").read_bytes()).hexdigest()
    ):
        raise ValueError("Audited local drawing plan must match the current immutable drawing")
    if args.output.exists() and any(args.output.iterdir()):
        raise ValueError("Use a new output directory; preserve previous trials")
    args.output.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(args.drawing_plan, args.output / "connection_plan.json")
    audit = args.drawing_plan.with_suffix(".audit.json")
    if not audit.exists():
        raise ValueError("Local inference audit required")
    shutil.copyfile(audit, args.output / "connection_plan.audit.json")
    report = {
        "success": False,
        "trial_state": "running",
        "parameters": {
            "wire_distance_mm": args.wire_distance_mm,
            "place_in_nest": args.place_in_nest,
            "support_approach_settle_time_s": 30,
            "nest_finger_envelope": "full_mechanical_stroke" if args.place_in_nest else None,
        },
        "world_sha256": hashlib.sha256(args.world.read_bytes()).hexdigest(),
        "model_sha256": {
            n: hashlib.sha256((args.models / (n + ".urdf")).read_bytes()).hexdigest()
            for n in ["left_arm", "right_arm"]
        },
        "scope": "Fixed P3: connector-first dual physical pickup and suspended wire support; no board insertion or all-connectors completion",
    }

    def checkpoint(phase):
        report["phase"] = phase
        session.phase = phase
        write_json_atomic(args.output / "progress.json", report)
        print(phase, flush=True)

    rclpy.init()
    session = None
    try:
        with (
            (args.output / "bridge.log").open("w") as log,
            gzip.open(args.output / "grip_observations.jsonl.gz", "wt") as guard_stream,
        ):
            session = ArmSession(args.world, args.models, log)
            session.wait_ready()
            guards = run(session, args, report, checkpoint, guard_stream)
            if args.place_in_nest:
                place_in_nest(session, args, report, checkpoint, guards)
    except Exception as error:
        if session:
            report["phase"] = session.phase
        report.update(error=str(error), planning_diagnostics=getattr(error, "diagnostics", None))
    finally:
        try:
            if session:
                session.close()
        except Exception as error:
            report.update(teardown_error=str(error), success=False)
        finally:
            if session:
                report.update(
                    contacts=list(session.contacts.values()),
                    unintended_contacts=session.unintended,
                    stop_diagnostics=session.stop_diagnostics,
                    preflights=session.preflights,
                )
                (args.output / "trajectory.json.gz").write_bytes(
                    gzip.compress(json.dumps(session.history).encode(), mtime=0)
                )
            report["trial_state"] = "finished"
            write_json_atomic(args.output / "validation.json", report)
            rclpy.shutdown()
    print(json.dumps(report, indent=2), flush=True)
    if not report["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
