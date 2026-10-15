"""Physical top regrasp after independently verified nest seating and parking."""

import hashlib
import json
import subprocess
import time

import numpy as np
from arm_collision_bounds import first_overlap
from arm_observation import acquire_observation
from cooperative_pick import shared_lift
from cooperative_transfer import rigid_pose
from evidence_io import write_json_atomic
from grasp_validation import assess_hold, assess_opening, assess_preclose_pose
from grip_guard import register_grip_guard
from pick_wire import evaluation_state
from preclose_alignment import correction_budget, correction_path, tracking_compensated_goal
from processes import stop_process
from seat_guard import end_nest_guard_for_pickup
from top_pickup_plan import full_stroke_bounds


def vision_body(observation):
    connectors = observation["connectors"]
    if len(connectors) != 1 or connectors[0]["plug_id"] != "P3":
        raise ValueError("Exactly one fresh P3 observation required")
    pose = connectors[0]["pose_6d"]
    body = np.eye(4)
    body[:3, :3], body[:3, 3] = pose["rotation_world_from_connector"], pose["center_xyz_m"]
    return rigid_pose(body)


def live_top_plan(session, args, report, body):
    row = session.tick()
    request = args.output / "top_pickup.request.json"
    output = args.output / "top_pickup.plan.json"
    if request.exists() or output.exists():
        raise ValueError("Preserve previous top pickup planning inputs")
    write_json_atomic(
        request,
        {
            "primary_arm": "left_arm",
            "pose_source": "fresh_rgbd",
            "vision_connector_pose": rigid_pose(body).tolist(),
            "robot_feedback": {"sim_time_s": row["sim_time_s"], "arms": row["arms"]},
        },
    )
    with (args.output / "top_pickup.plan.log").open("w") as log:
        process = subprocess.Popen(
            [
                "python3",
                "scripts/top_pickup_plan.py",
                "--world",
                str(args.world),
                "--models",
                str(args.models),
                "--request",
                str(request),
                "--output",
                str(output),
            ],
            stdout=log,
            stderr=log,
            start_new_session=True,
        )
        try:
            deadline = time.monotonic() + 180
            while process.poll() is None:
                if time.monotonic() > deadline:
                    raise RuntimeError("Top pickup planner compute budget exhausted")
                session.tick()
            if process.returncode:
                raise RuntimeError("Top pickup planner rejected; inspect saved plan/log")
            plan = json.loads(output.read_text())
            if (
                plan.get("request_sha256") != hashlib.sha256(request.read_bytes()).hexdigest()
                or plan.get("world_sha256") != report["world_sha256"]
                or plan.get("model_sha256") != report["model_sha256"]
                or not plan.get("success")
            ):
                raise RuntimeError("Top pickup plan provenance or outcome mismatch")
            return plan
        finally:
            stop_process(process, group=True)


def top_pickup(session, args, report, checkpoint, nest_guard, grip_stream):
    if not report.get("nest_aftercare_complete"):
        raise RuntimeError("Actual successful nest aftercare required before top regrasp")
    report.update(
        success=False,
        top_pickup_started=True,
        scope="Fixed P3 physical top regrasp; not board insertion or all-connectors completion",
    )
    name, prefix = "left_arm", "P3_top_primary"
    if prefix in report.get("grip_references", {}):
        raise ValueError("Preserve original top-grip reference")
    checkpoint("top_observation")
    observation = acquire_observation(
        session, args.output / "top_observation", calibration=args.nest_calibration
    )
    body = vision_body(observation)
    checkpoint("top_planning")
    plan = live_top_plan(session, args, report, body)
    report["top_pickup_plan_success"] = True
    relation = rigid_pose(plan["connector_from_tool"])
    for phase in ("hover", "approach"):
        checkpoint("top_" + phase)
        session.move(
            "top_" + phase,
            {n: np.asarray(path) for n, path in plan["paths"][phase].items()},
            full_finger_stroke=True,
            cartesian_speed_m_s=0.02 if phase == "approach" else None,
        )
    budget = {}
    for attempt in range(3):
        label = "top_preclose_" + str(attempt)
        checkpoint(label)
        observation = acquire_observation(
            session, args.output / label, calibration=args.nest_calibration
        )
        body = vision_body(observation)
        detected = observation["connectors"][0]["pose_6d"]
        actual = session.snapshot()["arms"][name]
        tool = np.eye(4)
        tool[:3, :3], tool[:3, 3] = actual["tcp_rotation_world"], actual["tcp"]
        check = assess_preclose_pose(detected, tool, relation)
        check["preferred_alignment"] = (
            check["position_error_m"] < 0.0005 and check["angle_error_rad"] < 0.005
        )
        report.setdefault("top_preclose_checks", []).append(check)
        if check["success"] and check["preferred_alignment"]:
            break
        if attempt == 2:
            raise RuntimeError("Top preclose rejected after bounded corrections")
        desired = body @ relation
        target = tracking_compensated_goal(
            desired, tool, session.arms[name].forward(session.targets[name])
        )
        q = np.asarray(actual["q_rad"])
        budget = correction_budget(budget, {name: session.arms[name].forward(q)}, {name: target})
        report["top_correction_budget"] = budget
        session.move(
            "top_correction",
            {name: correction_path(session.arms[name], q, target)},
            full_finger_stroke=True,
        )
    row = session.tick()
    joints = {n: np.asarray(state["q_rad"]) for n, state in row["arms"].items()}
    bounds = full_stroke_bounds(session.bounds)
    closure = {
        "arms": first_overlap(bounds, joints),
        "fixtures": session.fixtures.first_overlap(bounds, joints, margin=0.001),
    }
    report["top_actual_closure_preflight"] = closure
    if any(value is not None for value in closure.values()):
        raise RuntimeError("Corrected top pickup closing stroke rejected")
    baseline = evaluation_state(row, name, "P3")
    checkpoint("top_close")
    session.grips[name] = 0
    closed_row = session.dwell("top_close", 4)[-1]
    closed = evaluation_state(closed_row, name, "P3")
    rest = report["rest_gaps_m"][name]
    report["top_closure"] = assess_opening([closed["fingers"]], rest_gap_m=rest)
    if not report["top_closure"]["success"]:
        raise RuntimeError("Top closure rejected")
    grip_guard = register_grip_guard(
        session, report, prefix, closed, lambda row: evaluation_state(row, name, "P3"), grip_stream
    )
    end_nest_guard_for_pickup(session, report, "P3_nest", nest_guard, prefix, grip_guard)
    _, lifts = shared_lift({name: session.arms[name]}, {name: np.asarray(closed["q_rad"])})
    checkpoint("top_lift")
    session.move("top_lift", lifts, full_finger_stroke=True, cartesian_speed_m_s=0.02)
    checkpoint("top_hold")
    rows = session.dwell("top_hold", 4)
    report["top_hold"] = assess_hold(
        baseline, [evaluation_state(row, name, "P3") for row in rows], closed, rest_gap_m=rest
    )
    if not report["top_hold"]["success"]:
        raise RuntimeError("Top holding rejected")
    report["top_pickup_complete"] = True
    return relation, grip_guard
