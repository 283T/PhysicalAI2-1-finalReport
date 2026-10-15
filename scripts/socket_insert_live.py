"""Actual first-connector insertion after verified physical top regrasp."""

import hashlib
import json
import subprocess
import time

import numpy as np
from arm_observation import acquire_observation
from assembly_case import validate_connection_plan
from carried_contact_guard import register_carried_contact_guard
from cooperative_transfer import rigid_pose, single_arm_path
from evidence_io import write_json_atomic
from grasp_validation import assess_preclose_pose
from grip_guard import end_grip_for_release
from gripper_model import OPEN_TRAVEL_M
from held_wire_nest import seat_check
from pick_wire import evaluation_state
from processes import stop_process
from seat_guard import register_seat_guard
from top_pickup_live import vision_body


def live_socket_plan(session, args, report, relation):
    row = session.tick()
    request, output = args.output / "socket.request.json", args.output / "socket.plan.json"
    if request.exists() or output.exists():
        raise ValueError("Preserve previous insertion planning inputs")
    write_json_atomic(
        request,
        {
            "primary_arm": "left_arm",
            "plug_id": "P3",
            "pose_source": "fresh_rgbd",
            "vision_connector_from_tool": rigid_pose(relation).tolist(),
            "robot_feedback": {"sim_time_s": row["sim_time_s"], "arms": row["arms"]},
        },
    )
    with (args.output / "socket.plan.log").open("w") as log:
        process = subprocess.Popen(
            [
                "python3",
                "scripts/socket_plan_worker.py",
                "--world",
                str(args.world),
                "--models",
                str(args.models),
                "--request",
                str(request),
                "--drawing-plan",
                str(args.drawing_plan),
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
                    raise RuntimeError("Insertion planner compute budget exhausted")
                session.tick()
            if process.returncode:
                raise RuntimeError("Insertion planning rejected; inspect saved plan/log")
            plan = json.loads(output.read_text())
            expected = {
                "request_sha256": hashlib.sha256(request.read_bytes()).hexdigest(),
                "world_sha256": report["world_sha256"],
                "model_sha256": report["model_sha256"],
                "board_sha256": report["board_sha256"],
                "drawing_plan_sha256": report["drawing_plan_sha256"],
            }
            if not plan.get("success") or any(plan.get(k) != v for k, v in expected.items()):
                raise RuntimeError("Insertion plan provenance or outcome mismatch")
            return plan
        finally:
            stop_process(process, group=True)


def insert_after_top_pickup(
    session, args, report, checkpoint, relation, grip_guard, seat_stream, contact_stream
):
    if not report.get("top_pickup_complete") or not report.get("top_hold", {}).get("success"):
        raise RuntimeError("Actual physical top lift and stable hold required before insertion")
    report.update(
        success=False,
        insertion_started=True,
        scope="Fixed P3 contact insertion, release and withdrawal; not all-connectors or final matrix completion",
    )
    board = json.loads((args.world.parent / "board.json").read_text())
    rows = validate_connection_plan(json.loads(args.drawing_plan.read_text()), board)
    matching = [r for r in rows if r["plug_id"] == "P3"]
    if len(matching) != 1:
        raise ValueError("One drawing-selected P3 socket required")
    contact = register_carried_contact_guard(
        session, "P3", matching[0]["socket"], report, contact_stream
    )
    checkpoint("socket_held_observation")
    observation = acquire_observation(
        session, args.output / "socket_held_observation", calibration=args.nest_calibration
    )
    body = vision_body(observation)
    state = session.snapshot()["arms"]["left_arm"]
    tool = np.eye(4)
    tool[:3, :3], tool[:3, 3] = state["tcp_rotation_world"], state["tcp"]
    report["socket_held_pose_check"] = assess_preclose_pose(
        observation["connectors"][0]["pose_6d"], tool, relation
    )
    if not report["socket_held_pose_check"]["success"]:
        raise RuntimeError("Fresh held connector pose rejected before insertion planning")
    observed_relation = np.linalg.inv(body) @ tool
    checkpoint("socket_planning")
    plan = live_socket_plan(session, args, report, observed_relation)
    if plan["drawing_row"] != matching[0]:
        raise RuntimeError("Insertion plan changed the validated drawing row")
    goal = rigid_pose(plan["goal_connector_pose"])
    checkpoint("socket_hover")
    session.move(
        "socket_hover",
        {n: np.asarray(path) for n, path in plan["paths"]["hover"].items()},
        full_finger_stroke=True,
    )
    checkpoint("socket_insert")
    row = session.tick()
    contact.begin_descent(row["sim_time_s"])
    session.move(
        "socket_insert",
        {n: np.asarray(path) for n, path in plan["paths"]["insert"].items()},
        full_finger_stroke=True,
        cartesian_speed_m_s=0.005,
    )
    checkpoint("socket_before_release")
    report["socket_before_release"] = seat_check(session.dwell("socket_before_release", 2.5), goal)
    contact.require_target_contact()
    if not report["socket_before_release"]["success"]:
        raise RuntimeError("Connector not independently seated before physical release")
    connected = register_seat_guard(
        session,
        report,
        "P3_connected",
        goal,
        lambda row: evaluation_state(row, "left_arm", "P3"),
        seat_stream,
        kind="connected",
    )
    checkpoint("socket_open")
    end_grip_for_release(session, report, "P3_top_primary", grip_guard)
    session.grips["left_arm"] = OPEN_TRAVEL_M
    session.dwell("socket_open", 3)
    row = session.tick()
    q = np.asarray(row["arms"]["left_arm"]["q_rad"])
    retreat = session.arms["left_arm"].forward(q)
    retreat[2, 3] += 0.08
    path = single_arm_path(session.arms["left_arm"], q, retreat)
    checkpoint("socket_withdraw")
    session.move(
        "socket_withdraw", {"left_arm": path}, full_finger_stroke=True, cartesian_speed_m_s=0.02
    )
    checkpoint("socket_after_release")
    report["socket_after_release"] = seat_check(session.dwell("socket_after_release", 2.5), goal)
    contact.require_target_contact()
    if not report["socket_after_release"]["success"]:
        raise RuntimeError("Connector unstable after physical opening and withdrawal")
    report["single_connector_complete"] = True
    return connected, contact
