"""Compare sampled native tool feedback with saved commands, preserving trial failure."""

import argparse
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np
from arm_kinematics import rotation_error
from evidence_io import write_json_atomic
from feedback_pacing import pose_from_feedback


def analyze(pickup):
    report = json.loads((pickup / "validation.json").read_text())
    plan = json.loads((pickup / "nest_plan.json").read_text())
    if report.get("trial_state") != "finished":
        raise ValueError("Finished trial required")
    phase = report["phase"]
    index = int(phase.removeprefix("nest_transfer_"))
    order = list(plan["attempts"][plan["selected_attempt"]]["phases"][index]["joint_paths"])
    recorded = next(row["feedback_pacing"] for row in report["preflights"] if row["phase"] == phase)
    rows = json.loads(gzip.decompress((pickup / "trajectory.json.gz").read_bytes()))
    rows = [r for r in rows if r["phase"] == phase]
    if len(order) != 2 or not rows:
        raise ValueError("Two-tool path and sampled native rows required")
    result = {
        "scope": "Sampled trajectory rows compared with their saved commands; not every pacer call. Native pose and joint feedback may be asynchronous. Does not identify torque, contact forces, or root cause; does not alter success or gates.",
        "trial_success": report["success"], "phase": phase,
        "pacer_tool_order": order, "sample_count": len(rows),
        "first_sim_time_s": rows[0]["sim_time_s"], "last_sim_time_s": rows[-1]["sim_time_s"],
        "recorded_pacer_summary": recorded,
        "arms": {n: {"position_exceedances": 0, "angle_exceedances": 0,
                     "joint_exceedances": 0, "maximum_position_m": 0.0,
                     "maximum_angle_rad": 0.0, "maximum_joint_rad": 0.0} for n in order},
        "relative": {"position_exceedances": 0, "angle_exceedances": 0,
                     "maximum_position_m": 0.0, "maximum_angle_rad": 0.0},
    }
    for row in rows:
        actual, command = {}, {}
        for name in order:
            state = row["arms"][name]
            actual[name] = pose_from_feedback(state)
            command[name] = np.array(state["command_tcp_pose"])
            differences = {
                "position": float(np.linalg.norm(actual[name][:3, 3] - command[name][:3, 3])),
                "angle": float(np.linalg.norm(rotation_error(actual[name][:3, :3], command[name][:3, :3]))),
                "joint": float(np.max(np.abs(np.array(state["q_rad"]) - state["command_q_rad"]))),
            }
            for key, value in differences.items():
                unit = "m" if key == "position" else "rad"
                result["arms"][name][key + "_exceedances"] += int(value > recorded[key + "_gate_" + unit])
                maximum = "maximum_" + key + "_" + unit
                result["arms"][name][maximum] = max(result["arms"][name][maximum], value)
        a, b = order
        observed = np.linalg.inv(actual[a]) @ actual[b]
        desired = np.linalg.inv(command[a]) @ command[b]
        values = {
            "position": float(np.linalg.norm(observed[:3, 3] - desired[:3, 3])),
            "angle": float(np.linalg.norm(rotation_error(observed[:3, :3], desired[:3, :3]))),
        }
        for key, value in values.items():
            unit = "m" if key == "position" else "rad"
            result["relative"][key + "_exceedances"] += int(value > recorded["relative_" + key + "_gate_" + unit])
            maximum = "maximum_" + key + "_" + unit
            result["relative"][maximum] = max(result["relative"][maximum], value)
    result["input_sha256"] = {name: hashlib.sha256((pickup / name).read_bytes()).hexdigest()
                              for name in ("validation.json", "trajectory.json.gz", "nest_plan.json")}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pickup", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Preserve previous diagnostics; use a new output path")
    result = analyze(args.pickup)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_json_atomic(args.output, result)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
