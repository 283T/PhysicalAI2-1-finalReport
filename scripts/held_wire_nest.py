"""Live cooperative nest placement; object poses are evaluation only."""

import json
import subprocess
import time

import numpy as np
from arm_observation import acquire_observation
from cooperative_transfer import rigid_pose, single_arm_path
from evidence_io import write_json_atomic
from grasp_validation import assess_preclose_pose, assess_seat
from grip_guard import end_grip_for_release
from gripper_model import OPEN_TRAVEL_M
from pick_wire import evaluation_state
from processes import stop_process


def live_plan(session, args, row, relation):
    request = args.output / "nest_plan_request.json"
    output = args.output / "nest_plan.json"
    write_json_atomic(
        request,
        {
            "robot_feedback": {"sim_time_s": row["sim_time_s"], "arms": row["arms"]},
            "vision_connector_from_tool": relation.tolist(),
            "full_finger_stroke": True,
        },
    )
    with (args.output / "nest_planner.log").open("w") as log:
        process = subprocess.Popen(
            [
                "python3",
                "scripts/plan_held_wire_transfer.py",
                "--world",
                str(args.world),
                "--models",
                str(args.models),
                "--feedback-request",
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
                    raise RuntimeError("Live nest planning time budget exhausted")
                session.tick()
            if process.returncode:
                raise RuntimeError("Nest planner failed; see nest_planner.log")
            return json.loads(output.read_text())
        finally:
            stop_process(process, group=True)


def seat_check(rows, goal):
    result = assess_seat(
        [evaluation_state(row, "right_arm", "P3") for row in rows],
        center=goal[:3, 3],
        rotation=goal[:3, :3],
    )
    result["minimum_required_duration_s"] = 2.0
    result["success"] = result["success"] and result.get("duration_s", 0) >= 2.0
    return result


def place_in_nest(session, args, report, checkpoint, guards):
    # Do not let a successful pickup make a failed subsequent placement succeed.
    report.update(
        dual_pickup_success=True,
        success=False,
        scope="Fixed P3: dual contact pickup, fresh held RGB-D, cooperative nest transfer, primary opening and retreat. No top regrasp, board insertion or all-connectors completion",
    )
    checkpoint("nest_held_observation")
    fresh = acquire_observation(
        session, args.output / "nest_held_observation", calibration=args.inspection_calibration
    )
    pose = fresh["connectors"][0]["pose_6d"]
    row = session.snapshot()
    tool = np.eye(4)
    tool[:3, :3], tool[:3, 3] = (
        row["arms"]["right_arm"]["tcp_rotation_world"],
        row["arms"]["right_arm"]["tcp"],
    )
    report["nest_held_pose_check"] = assess_preclose_pose(
        pose, tool, np.array(report["primary_connector_from_tool"])
    )
    if not report["nest_held_pose_check"]["success"]:
        raise RuntimeError("Fresh held connector/tool relation rejected")
    body = np.eye(4)
    body[:3, :3], body[:3, 3] = pose["rotation_world_from_connector"], pose["center_xyz_m"]
    relation = np.linalg.inv(rigid_pose(body)) @ rigid_pose(tool)
    checkpoint("nest_planning")
    plan = live_plan(session, args, row, relation)
    report["nest_plan_success"] = plan["success"]
    if not plan["success"]:
        raise RuntimeError("No continuous screened cooperative nest transfer")
    for phase in plan["attempts"][plan["selected_attempt"]]["phases"]:
        label = "nest_transfer_" + str(phase["index"])
        checkpoint(label)
        session.move(
            label,
            {name: np.array(path) for name, path in phase["joint_paths"].items()},
            cartesian_speed_m_s=0.02,
            full_finger_stroke=True,
        )
    goal = np.array(plan["goal_connector_pose"])
    checkpoint("nest_before_release")
    report["nest_before_release"] = seat_check(session.dwell("nest_before_release", 2.5), goal)
    if not report["nest_before_release"]["success"]:
        raise RuntimeError("Nest alignment unstable before release")
    checkpoint("nest_primary_open")
    end_grip_for_release(session, report, "primary", guards["primary"])
    session.grips["right_arm"] = OPEN_TRAVEL_M
    session.dwell("nest_primary_open", 3)
    row = session.snapshot()
    current = np.array(row["arms"]["right_arm"]["q_rad"])
    retreat = session.arms["right_arm"].forward(current)
    retreat[:3, 3] -= retreat[:3, 2] * 0.08
    path = single_arm_path(session.arms["right_arm"], current, retreat)
    checkpoint("nest_primary_retreat")
    session.move("nest_primary_retreat", {"right_arm": path}, cartesian_speed_m_s=0.02)
    checkpoint("nest_after_release")
    report["nest_seat"] = seat_check(session.dwell("nest_after_release", 2.5), goal)
    report["success"] = report["nest_seat"]["success"]
    if not report["success"]:
        raise RuntimeError("Nest seat failed after primary opening and retreat")
