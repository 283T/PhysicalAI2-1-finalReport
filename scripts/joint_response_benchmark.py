"""Isolated empty-gripper wrist response; never counts as assembly success."""

import argparse
import gzip
import hashlib
import json
import os
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from arm_session import ArmSession
from evidence_io import write_json_atomic
from processes import stop_process


def run(case, models, output, max_step_size=None):
    if output.exists() and any(output.iterdir()):
        raise ValueError("Use a new output directory")
    output.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(case / "world.sdf")
    world = tree.getroot().find("world")
    original_step = float(world.findtext("physics/max_step_size"))
    if max_step_size is not None:
        if not np.isfinite(max_step_size) or not 0 < max_step_size <= original_step:
            raise ValueError("Comparison timestep must be positive and no coarser than source")
        world.find("physics/max_step_size").text = str(max_step_size)
    for model in list(world.findall("model")):
        if model.get("name") not in ("left_arm", "right_arm", "fixtures", "conveyor_carrier"):
            world.remove(model)
    ET.indent(tree)
    path = output / "world.sdf"
    tree.write(path, encoding="utf-8", xml_declaration=True)
    report = {
        "trial_state": "running",
        "success": False,
        "scope": "Empty-gripper wrist response with original arms, fixtures and controller limits; harness and cameras removed. No grasp, transport or assembly success",
        "source_world_sha256": hashlib.sha256((case / "world.sdf").read_bytes()).hexdigest(),
        "world_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "model_sha256": {
            n: hashlib.sha256((models / (n + ".urdf")).read_bytes()).hexdigest()
            for n in ("left_arm", "right_arm")
        },
        "joint_index": 6,
        "targets_rad": [0.04, -0.04, 0.0],
        "source_max_step_size_s": original_step,
        "max_step_size_s": max_step_size or original_step,
        "steps": [],
    }
    os.environ["GZ_PARTITION"] = "wrist-response-" + str(os.getpid())
    os.environ["ROS_DOMAIN_ID"] = "84"
    write_json_atomic(output / "progress.json", report)
    session = server = None
    rclpy.init()
    try:
        with (
            (output / "server.log").open("w") as server_log,
            (output / "bridge.log").open("w") as bridge_log,
        ):
            server = subprocess.Popen(
                ["gz", "sim", "-s", "-r", str(path)],
                stdout=server_log,
                stderr=server_log,
                start_new_session=True,
            )
            session = ArmSession(path, models, bridge_log)
            session.wait_ready()
            session.dwell("initial_open", 2)
            for index, target in enumerate(report["targets_rad"]):
                label = "wrist_step_" + str(index)
                row = session.snapshot()
                paths = {}
                for name in session.arms:
                    start = np.array(row["arms"][name]["q_rad"])
                    goal = start.copy()
                    goal[5] = target
                    paths[name] = np.array([start, goal])
                session.move(label, paths)
                rows = session.dwell(label + "_hold", 5)
                window = [r for r in rows if r["sim_time_s"] >= rows[-1]["sim_time_s"] - 1]
                step = {"target_rad": target, "arms": {}}
                for name in session.arms:
                    errors = [abs(r["arms"][name]["q_rad"][5] - target) for r in window]
                    step["arms"][name] = {
                        "maximum_error_rad_last_second": float(max(errors)),
                        "final_error_rad": float(errors[-1]),
                        "within_joint_gate": bool(max(errors) < 0.01),
                        "samples": len(window),
                    }
                report["steps"].append(step)
                write_json_atomic(output / "progress.json", report)
            report["success"] = all(
                s["within_joint_gate"] for step in report["steps"] for s in step["arms"].values()
            )
    except Exception as error:
        report["error"] = str(error)
    finally:
        try:
            if session:
                session.close()
        except Exception as error:
            report.update(teardown_error=str(error), success=False)
        finally:
            if server:
                stop_process(server, group=True)
            if session:
                report.update(
                    phase=session.phase,
                    stop_diagnostics=session.stop_diagnostics,
                    preflights=session.preflights,
                    contacts=list(session.contacts.values()),
                    unintended_contacts=session.unintended,
                )
                (output / "trajectory.json.gz").write_bytes(
                    gzip.compress(json.dumps(session.history).encode(), mtime=0)
                )
            report["trial_state"] = "finished"
            write_json_atomic(output / "validation.json", report)
            rclpy.shutdown()
    print(json.dumps(report, indent=2))
    if not report["success"]:
        raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("case", "models", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--max-step-size", type=float)
    args = parser.parse_args()
    run(args.case, args.models, args.output, args.max_step_size)


if __name__ == "__main__":
    main()
