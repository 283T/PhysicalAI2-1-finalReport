"""Isolated no-payload arm control comparison; never an assembly success test."""

import argparse
import hashlib
import json
import os
from pathlib import Path

import numpy as np
import rclpy
from arm_kinematics import rotation_error
from check_reach import verify_arm
from evidence_io import write_json_atomic
from plan_reach import load_arms


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("world", "models", "pickup", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Use a fresh output directory; preserve previous diagnostics")
    args.output.mkdir(parents=True)
    report = json.loads((args.pickup / "validation.json").read_text())
    if report.get("trial_state") != "finished":
        parser.error("Finished source trial required")
    commands = report["stop_diagnostics"]["last_observation"]["arms"]
    arms = load_arms(args.world, args.models)
    result = {
        "scope": "Separate single-arm worlds without harness, fixtures or the other arm. Recorded robot commands are goals, never restored object poses. Endpoint tracking only; not a replay of the loaded path, not causal proof and not assembly success.",
        "source_trial_success": report["success"],
        "source_phase": report["phase"],
        "input_sha256": {
            "validation": hashlib.sha256((args.pickup / "validation.json").read_bytes()).hexdigest(),
            "world": hashlib.sha256(args.world.read_bytes()).hexdigest(),
        },
        "gates": {"position_m": 0.001, "angle_rad": 0.01, "joint_rad": 0.01, "fk_m": 0.001},
        "arms": {},
    }
    os.environ["ROS_DOMAIN_ID"] = "173"
    rclpy.init()
    try:
        for name, arm in arms.items():
            folder = args.output / name
            folder.mkdir()
            q = commands[name]["command_q_rad"]
            goal = {"plug_id": "none", "stage": "unloaded_tracking",
                    "q_rad": q, "target_pose": arm.forward(q).tolist()}
            try:
                rows = verify_arm(name, arm, args.world, [goal], folder,
                                  tolerances=result["gates"])
                result["arms"][name] = rows[-1]
            except Exception as error:
                rows = json.loads((folder / "latest.json").read_text()) if (folder / "latest.json").exists() else []
                result["arms"][name] = {"success": False, "error": str(error),
                                         "observations": rows}
            write_json_atomic(args.output / "result.json", result)
    finally:
        rclpy.shutdown()
    if all("measured_tcp_pose" in row for row in result["arms"].values()):
        a, b = arms
        measured = np.linalg.inv(np.array(result["arms"][a]["measured_tcp_pose"])) @ np.array(result["arms"][b]["measured_tcp_pose"])
        target = np.linalg.inv(arms[a].forward(commands[a]["command_q_rad"])) @ arms[b].forward(commands[b]["command_q_rad"])
        result["separate_world_endpoint_pair"] = {
            "order": [a, b],
            "scope": "Combines endpoints measured in different worlds and at different times; not simultaneous dual-arm feedback",
            "relative_position_error_m": float(np.linalg.norm(measured[:3, 3] - target[:3, 3])),
            "relative_angle_error_rad": float(np.linalg.norm(rotation_error(measured[:3, :3], target[:3, :3]))),
        }
    result["all_isolated_endpoints_within_gates"] = all(r["success"] for r in result["arms"].values())
    write_json_atomic(args.output / "result.json", result)
    print(json.dumps(result, indent=2))
    if not result["all_isolated_endpoints_within_gates"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
