"""Gated physical support release and parking after actual passive-nest seating."""

import hashlib
import json
import subprocess
import time

import numpy as np
from evidence_io import write_json_atomic
from grip_guard import end_grip_for_release
from gripper_model import OPEN_TRAVEL_M
from held_wire_nest import seat_check
from pick_wire import evaluation_state
from processes import stop_process
from seat_guard import register_seat_guard


def live_parking_plan(session, args, report, moving):
    row = session.tick()
    base = args.output / ("nest_park_" + moving)
    request = base.with_suffix(".request.json")
    output = base.with_suffix(".plan.json")
    if output.exists():
        raise ValueError("Preserve previous parking plans")
    write_json_atomic(
        request,
        {
            "moving_arm": moving,
            "robot_feedback": {"sim_time_s": row["sim_time_s"], "arms": row["arms"]},
        },
    )
    with base.with_suffix(".log").open("w") as log:
        process = subprocess.Popen(
            [
                "python3",
                "scripts/park_arm_plan.py",
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
                    raise RuntimeError("Parking planner compute budget exhausted")
                session.tick()
            if process.returncode:
                raise RuntimeError("Parking planner rejected; see " + str(base.with_suffix(".log")))
            plan = json.loads(output.read_text())
            if (
                plan.get("request_sha256") != hashlib.sha256(request.read_bytes()).hexdigest()
                or plan.get("world_sha256") != report["world_sha256"]
                or plan.get("model_sha256") != report["model_sha256"]
                or plan.get("moving_arm") != moving
                or not plan.get("success")
            ):
                raise RuntimeError("Parking plan input provenance or outcome mismatch")
            return plan
        finally:
            stop_process(process, group=True)


def release_support_and_park(session, args, report, checkpoint, guards, seat_stream):
    if not report.get("nest_seat", {}).get("success") or not report.get("dual_pickup_success"):
        raise RuntimeError("Actual successful nest seating required before aftercare")
    report.update(
        success=False,
        nest_aftercare_started=True,
        scope="Fixed P3: actual nest seating, support release and arm parking; no top regrasp, insertion or all-connectors success",
    )
    goal = np.asarray(
        json.loads((args.output / "nest_plan.json").read_text())["goal_connector_pose"]
    )
    checkpoint("nest_support_open")
    monitor = register_seat_guard(
        session,
        report,
        "P3_nest",
        goal,
        lambda row: evaluation_state(row, "right_arm", "P3"),
        seat_stream,
        kind="passive_nest",
    )
    end_grip_for_release(session, report, "support", guards["support"])
    session.grips["left_arm"] = OPEN_TRAVEL_M
    report["nest_after_support_release"] = seat_check(session.dwell("nest_support_open", 3), goal)
    if not report["nest_after_support_release"]["success"]:
        raise RuntimeError("Nest unstable after physical wire support release")
    for moving in ("left_arm", "right_arm"):
        label = "nest_park_" + moving
        checkpoint(label + "_planning")
        plan = live_parking_plan(session, args, report, moving)
        report.setdefault("nest_parking_plans", {})[moving] = plan
        checkpoint(label)
        session.move(
            label, {n: np.asarray(q) for n, q in plan["paths"].items()}, full_finger_stroke=True
        )
        check = seat_check(session.dwell(label + "_seat_check", 2.5), goal)
        report.setdefault("nest_parking_seat_checks", {})[moving] = check
        if not check["success"]:
            raise RuntimeError("Nest unstable during released arm parking")
    report["nest_aftercare_complete"] = True
    return monitor
