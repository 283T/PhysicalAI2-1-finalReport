"""Verify native physical opening/withdrawal and the final fixed-P3 seat window."""

import argparse
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np
from assembly_case import validate_connection_plan
from cooperative_transfer import rigid_pose, socket_poses
from evidence_io import write_json_atomic
from grasp_validation import assess_opening, assess_seat
from gripper_model import MAX_TRAVEL_M, OPEN_TRAVEL_M, measured_rest_gap
from pick_wire import evaluation_state
from project_paths import ROOT
from seat_guard import seat_observation


def witness(rows, goal, *, rest_gap_m):
    """Current left-arm top-insertion sequence only; no command state is evidence."""
    try:
        goal = rigid_pose(goal)
        criteria = json.loads((ROOT / "configs/final_acceptance.json").read_text())[
            "connected_after_release"
        ]
        before = [r for r in rows if r["phase"] == "socket_before_release"]
        opening = [r for r in rows if r["phase"] == "socket_open"]
        withdrawing = [r for r in rows if r["phase"] == "socket_withdraw"]
        final = [r for r in rows if r["phase"] == "socket_after_release"]
        if not before or not opening or len(withdrawing) < 2 or not final:
            raise ValueError(
                "Before-release, physical opening and final seat observations required"
            )
        stamps = np.asarray([r["sim_time_s"] for r in rows], dtype=float)
        if not np.isfinite(stamps).all() or np.any(np.diff(stamps) < 0):
            raise ValueError("Finite, nondecreasing native trajectory clock required")
        if not (
            before[-1]["sim_time_s"] <= opening[0]["sim_time_s"]
            and opening[-1]["sim_time_s"] <= withdrawing[0]["sim_time_s"]
            and withdrawing[-1]["sim_time_s"] < final[0]["sim_time_s"]
        ):
            raise ValueError("Opening must physically precede withdrawal and final observation")
        closed = evaluation_state(before[-1], "left_arm", "P3")
        closure = assess_opening([closed["fingers"]], rest_gap_m=rest_gap_m)
        before_check = seat_observation(goal, closed, closed, criteria)
        open_values = np.asarray([r["arms"]["left_arm"]["fingers"] for r in opening], dtype=float)
        final_values = np.asarray([r["arms"]["left_arm"]["fingers"] for r in final], dtype=float)
        if (
            open_values.shape != (len(opening), 2)
            or final_values.shape != (len(final), 2)
            or not np.isfinite(open_values).all()
            or not np.isfinite(final_values).all()
        ):
            raise ValueError("Finite measurements of both physical fingers required")
        minimum_open = OPEN_TRAVEL_M - 0.0005
        maximum_open = MAX_TRAVEL_M + 0.0005
        opened = bool(
            np.any(np.all((open_values >= minimum_open) & (open_values <= maximum_open), axis=1))
        )
        stayed_open = bool(np.all((final_values >= minimum_open) & (final_values <= maximum_open)))
        before_tcp = np.asarray(closed["tcp"], dtype=float)
        final_tcp = np.asarray([r["arms"]["left_arm"]["tcp"] for r in final], dtype=float)
        if (
            before_tcp.shape != (3,)
            or final_tcp.shape != (len(final), 3)
            or not np.isfinite([*final_tcp, before_tcp]).all()
        ):
            raise ValueError("Finite native robot TCP observations required")
        withdrawal = float(np.min(final_tcp[:, 2] - before_tcp[2]))
        observed = [evaluation_state(r, "left_arm", "P3") for r in final]
        seat = assess_seat(observed, center=goal[:3, 3], rotation=goal[:3, :3])
        frozen_window_checks = [seat_observation(goal, observed[0], r, criteria) for r in observed]
        seat["success"] = bool(
            seat["success"]
            and all(check["success"] for check in frozen_window_checks)
            and seat.get("duration_s", 0) >= criteria["minimum_observation_duration_s"]
            and seat.get("samples", 0) >= criteria["minimum_samples"]
        )
        return {
            "success": bool(
                closure["success"]
                and before_check["success"]
                and opened
                and stayed_open
                and withdrawal >= 0.075
                and seat["success"]
            ),
            "before_release_closure": closure,
            "before_release_seat": before_check,
            "physical_opening_observed": opened,
            "remained_physically_open": stayed_open,
            "minimum_final_finger_travel_m": float(final_values.min()),
            "minimum_native_upward_withdrawal_m": withdrawal,
            "required_native_upward_withdrawal_m": 0.075,
            "native_withdrawal_samples": len(withdrawing),
            "seat_window": seat,
            "scope": "Native action witness for the current 80 mm withdrawal command (require at least 75 mm), plus the unchanged frozen final seat window. Not evidence of local-AI inference, grip stability, contact insertion or all-connectors completion.",
        }
    except (KeyError, ValueError, TypeError) as error:
        return {"success": False, "reason": str(error)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("pickup", "case", "models", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    report = json.loads((args.pickup / "validation.json").read_text())
    if report.get("trial_state") != "finished":
        parser.error("Only finished trials can be evaluated")
    world = args.case / "world.sdf"
    model = args.models / "left_arm.urdf"
    if (
        report.get("world_sha256") != hashlib.sha256(world.read_bytes()).hexdigest()
        or report.get("model_sha256", {}).get("left_arm")
        != hashlib.sha256(model.read_bytes()).hexdigest()
    ):
        parser.error("World or robot model differs from executed trial")
    plan = json.loads((args.pickup / "connection_plan.json").read_text())
    board = json.loads((args.case / "board.json").read_text())
    rows = validate_connection_plan(plan, board)
    selected = [r for r in rows if r["plug_id"] == "P3"]
    if len(selected) != 1:
        parser.error("Exactly one diagram-selected P3 connection required")
    goal = socket_poses(board, selected[0], np.eye(4))["seated"]
    with gzip.open(args.pickup / "trajectory.json.gz", "rt") as stream:
        trajectory = json.load(stream)
    result = witness(trajectory, goal, rest_gap_m=measured_rest_gap(model, "left_arm_"))
    result.update(
        trial_success=report.get("success", False),
        goal_pose_from_drawing=goal.tolist(),
        final_acceptance_eligible=False,
        input_sha256={
            str(p): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (
                world,
                model,
                args.case / "board.json",
                args.pickup / "connection_plan.json",
                args.pickup / "validation.json",
                args.pickup / "trajectory.json.gz",
                ROOT / "configs/final_acceptance.json",
            )
        },
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_json_atomic(args.output, result)
    print(json.dumps(result, indent=2))
    if not result["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
