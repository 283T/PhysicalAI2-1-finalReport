"""Fresh RGB-D wire contact pickup with a MELFA arm and screened parked arm."""

import argparse
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np
import rclpy
from arm_kinematics import quaternion_rotation, rotation_error
from arm_observation import acquire_observation
from arm_session import ArmSession
from cartesian_path import translation_path
from evidence_io import write_json_atomic
from grasp_validation import assess_hold, assess_opening, world_link_positions
from gripper_model import measured_rest_gap
from plan_wire_pickup import plan_wire_pickup, upright_wire_target


def acquire(session, output, distance, calibration=None):
    return acquire_observation(session, output, wire_distance_mm=distance, calibration=calibration)


def evaluation_state(row, arm, link):
    poses = row["evaluation_poses"]
    rotation = quaternion_rotation(poses["wire_harness_multi"]["q"]) @ quaternion_rotation(
        poses[link]["q"]
    )
    return {
        "time_s": row["sim_time_s"],
        "plug": world_link_positions(poses, "wire_harness_multi", [link])[link].tolist(),
        "plug_rotation_world": rotation.tolist(),
        **row["arms"][arm],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--models", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--wire-distance-mm", type=float, default=170)
    parser.add_argument(
        "--inspection-calibration", type=Path, default=Path("configs/end_inspection_camera.json")
    )
    args = parser.parse_args()
    if not np.isfinite(args.wire_distance_mm) or not 75 <= args.wire_distance_mm <= 200:
        parser.error("Finite support distance between 75 and 200 mm required")
    args.output.mkdir(parents=True, exist_ok=True)
    for name in ("validation.json", "trajectory.json.gz"):
        (args.output / name).unlink(missing_ok=True)
    report = {
        "success": False,
        "trial_state": "running",
        "phase": "startup",
        "world_sha256": hashlib.sha256(args.world.read_bytes()).hexdigest(),
        "model_sha256": {
            n: hashlib.sha256((args.models / (n + ".urdf")).read_bytes()).hexdigest()
            for n in ("left_arm", "right_arm")
        },
        "scope": "Fresh RGB-D P3 wire pickup by left MELFA, right arm parked; no connector pickup, dual-arm holding, transport, insertion or release validation",
    }

    def checkpoint(phase):
        report["phase"] = phase
        write_json_atomic(args.output / "progress.json", report)
        print(phase, flush=True)

    checkpoint("startup")
    rclpy.init()
    session = None
    try:
        with (args.output / "bridge.log").open("w") as log:
            session = ArmSession(args.world, args.models, log)
            session.wait_ready()
            rest = measured_rest_gap(args.models / "left_arm.urdf", "left_arm_")
            if rest >= 0.008:
                raise RuntimeError("Support fingers cannot pinch an 8 mm wire")
            checkpoint("observe_wire")
            observation = acquire(session, args.output / "observations", args.wire_distance_mm)
            current = {n: s["q_rad"] for n, s in session.snapshot()["arms"].items()}
            plan = plan_wire_pickup(
                session.arms,
                session.bounds,
                args.world,
                observation,
                current,
                "left_arm",
                "P3",
                upright=True,
                include_carrier=True,
            )
            write_json_atomic(args.output / "wire_plan.json", plan)
            if not plan["success"]:
                raise RuntimeError("Wire pickup preflight rejected")
            for phase in ("hover", "approach"):
                checkpoint(phase)
                session.move(phase, plan["paths"][phase])
            checkpoint("preclose_observation")
            fresh = acquire(
                session,
                args.output / "preclose",
                args.wire_distance_mm,
                args.inspection_calibration,
            )
            before, after = observation["wire_supports"][0], fresh["wire_supports"][0]
            desired = np.asarray(plan["grasp_tcp_pose"])
            actual = session.snapshot()["arms"]["left_arm"]
            center_error = float(
                np.linalg.norm(np.array(before["wire_center_xyz_m"]) - after["wire_center_xyz_m"])
            )
            tangent_error = float(
                np.arccos(
                    np.clip(abs(np.dot(before["tangent_world"], after["tangent_world"])), 0, 1)
                )
            )
            position_error = float(np.linalg.norm(np.array(actual["tcp"]) - desired[:3, 3]))
            angle_error = float(
                np.linalg.norm(
                    rotation_error(desired[:3, :3], np.array(actual["tcp_rotation_world"]))
                )
            )
            report["preclose"] = {
                "wire_center_change_m": center_error,
                "wire_tangent_change_rad": tangent_error,
                "tool_position_error_m": position_error,
                "tool_angle_error_rad": angle_error,
                "success": center_error < 0.002
                and tangent_error < 0.025
                and position_error < 0.002
                and angle_error < 0.025,
            }
            checkpoint("preclose_checked")
            if not report["preclose"]["success"]:
                raise RuntimeError("Fresh preclose wire alignment rejected")
            # Identify the physical segment for independent evaluation only.
            baseline = session.snapshot()
            names = [n for n in baseline["evaluation_poses"] if n.startswith("P3_wire_")]
            positions = world_link_positions(
                baseline["evaluation_poses"], "wire_harness_multi", names
            )
            target = min(
                names, key=lambda n: np.linalg.norm(positions[n] - before["wire_center_xyz_m"])
            )
            report["evaluation_link"] = target
            baseline_state = evaluation_state(baseline, "left_arm", target)
            checkpoint("close")
            session.grips["left_arm"] = 0
            closed = session.dwell("close", 4)[-1]
            expected_width = upright_wire_target(after)[1]
            report["expected_wire_width_m"] = expected_width
            report["closure"] = assess_opening(
                [closed["arms"]["left_arm"]["fingers"]], expected_width, rest_gap_m=rest
            )
            if not report["closure"]["success"]:
                raise RuntimeError("Wire closure rejected")
            q = np.array(closed["arms"]["left_arm"]["q_rad"])
            goal = session.arms["left_arm"].forward(q)
            goal[2, 3] += 0.1
            checkpoint("lift")
            session.move("lift", {"left_arm": translation_path(session.arms["left_arm"], q, goal)})
            checkpoint("hold")
            held = session.dwell("hold", 4)
            report["hold"] = assess_hold(
                baseline_state,
                [evaluation_state(s, "left_arm", target) for s in held],
                evaluation_state(closed, "left_arm", target),
                housing_width=expected_width,
                rest_gap_m=rest,
            )
            report["success"] = report["hold"]["success"]
            if not report["success"]:
                raise RuntimeError("Wire hold rejected")
    except Exception as error:
        report["error"] = str(error)
        report["planning_diagnostics"] = getattr(error, "diagnostics", None)
    finally:
        try:
            if session:
                session.close()
        except Exception as error:
            report["teardown_error"] = str(error)
            report["success"] = False
        finally:
            if session:
                report["stop_diagnostics"] = session.stop_diagnostics
                report["contact_publishers"] = session.contact_publishers
                report["expected_contact_publishers"] = session.expected_contact_publishers
                report["contact_messages"] = session.contact_count
                report["contacts"] = list(session.contacts.values())
                report["unintended_contacts"] = session.unintended
                report["preflights"] = session.preflights
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
