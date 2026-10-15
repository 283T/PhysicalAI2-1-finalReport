"""Separate recorded grip rotation from robot tracking in a completed trial."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from arm_kinematics import rotation_error
from evidence_io import write_json_atomic
from evidence_stream import evidence_digest, open_evidence
from feedback_pacing import pose_from_feedback


def analyze(pickup):
    report = json.loads((pickup / "validation.json").read_text())
    if report["trial_state"] != "finished":
        raise ValueError("Only completed trials may be analyzed")
    first, last = {}, {}
    with open_evidence(pickup / "grip_observations.jsonl.gz") as stream:
        for line in stream:
            row = json.loads(line)
            if row["phase"] == report["phase"]:
                first.setdefault(row["grip"], row)
                last[row["grip"]] = row
    if set(last) != {"primary", "support"}:
        raise ValueError("Two monitored grips required in the terminal phase")
    if any(
        rows["primary"]["sim_time_s"] != rows["support"]["sim_time_s"] for rows in (first, last)
    ):
        raise ValueError("Time-matched tool observations required")
    result = {
        "scope": "Independent recorded tracking diagnostic. Rotation-vector projection, not torque measurement, causal proof or a success criterion",
        "phase": report["phase"],
        "input_sha256": {
            "validation.json": hashlib.sha256(
                (pickup / "validation.json").read_bytes()
            ).hexdigest(),
            "grip_observations.jsonl.gz": evidence_digest(pickup / "grip_observations.jsonl.gz"),
        },
    }
    for name, row in last.items():
        observed, reference = row["observed"], report["grip_references"][name]
        old = np.array(reference["tcp_rotation_world"]).T @ reference["plug_rotation_world"]
        new = np.array(observed["tcp_rotation_world"]).T @ observed["plug_rotation_world"]
        vector = rotation_error(new, old)
        axis = old[:, 0]
        component = float(np.dot(vector, axis))
        command = np.array(observed["command_tcp_pose"])
        pose = pose_from_feedback(observed)
        result[name] = {
            "sim_time_s": row["sim_time_s"],
            "grasp_rotation_rad": float(np.linalg.norm(vector)),
            "local_x_rotation_component_rad": abs(component),
            "perpendicular_rotation_component_rad": float(
                np.linalg.norm(vector - component * axis)
            ),
            "tcp_tracking_position_m": float(np.linalg.norm(pose[:3, 3] - command[:3, 3])),
            "tcp_tracking_angle_rad": float(
                np.linalg.norm(rotation_error(pose[:3, :3], command[:3, :3]))
            ),
        }

    def relative(rows, commands=False):
        poses = [
            np.array(rows[n]["observed"]["command_tcp_pose"])
            if commands
            else pose_from_feedback(rows[n]["observed"])
            for n in ("primary", "support")
        ]
        return np.linalg.inv(poses[0]) @ poses[1]

    actual, command, start = relative(last), relative(last, commands=True), relative(first)
    result["tool_pair"] = {
        "sim_time_s": last["primary"]["sim_time_s"],
        "actual_relative_position_change_m": float(np.linalg.norm(actual[:3, 3] - start[:3, 3])),
        "actual_relative_angle_change_rad": float(
            np.linalg.norm(rotation_error(actual[:3, :3], start[:3, :3]))
        ),
        "relative_tracking_position_m": float(np.linalg.norm(actual[:3, 3] - command[:3, 3])),
        "relative_tracking_angle_rad": float(
            np.linalg.norm(rotation_error(actual[:3, :3], command[:3, :3]))
        ),
    }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pickup", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.pickup)
    write_json_atomic(args.output, result)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
