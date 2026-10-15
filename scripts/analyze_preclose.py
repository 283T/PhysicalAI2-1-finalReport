"""Post-trial image/tool alignment diagnosis; truth is evaluation only."""

import argparse
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np
from arm_kinematics import quaternion_rotation, rotation_error
from grasp_validation import world_link_positions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pickup", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = args.pickup
    if args.output.exists():
        raise ValueError("Preserve earlier diagnosis; use a new output file")
    if json.loads((root / "validation.json").read_text()).get("trial_state") != "finished":
        raise ValueError("Only diagnose finished trials")
    rows = json.loads(gzip.decompress((root / "trajectory.json.gz").read_bytes()))
    report = []
    # Original image-only relation, as used by primary_pickup.
    d0 = json.loads((root / "primary_observation/detections.json").read_text())["connectors"][0][
        "pose_6d"
    ]
    b0 = np.eye(4)
    b0[:3, :3] = d0["rotation_world_from_connector"]
    b0[:3, 3] = d0["center_xyz_m"]
    relation = np.linalg.inv(b0) @ np.array(d0["grasp_tcp_pose"])
    observations = sorted(root.glob("primary_preclose_*/detections.json"))
    if not rows or not observations:
        raise ValueError("Recorded robot trajectory and preclose observations required")
    for p in observations:
        i = int(p.parent.name.rsplit("_", 1)[1])
        obs = json.loads(p.read_text())
        t = obs["sensor_stamp_ns"] / 1e9
        r = min(rows, key=lambda r: abs(r["sim_time_s"] - t))
        poses = r["evaluation_poses"]
        native = np.eye(4)
        native[:3, :3] = quaternion_rotation(
            poses["wire_harness_multi"]["q"]
        ) @ quaternion_rotation(poses["P3"]["q"])
        native[:3, 3] = world_link_positions(poses, "wire_harness_multi", ["P3"])["P3"]
        image = obs["connectors"][0]["pose_6d"]
        vision = np.eye(4)
        vision[:3, :3] = image["rotation_world_from_connector"]
        vision[:3, 3] = image["center_xyz_m"]
        s = r["arms"]["right_arm"]
        tool = np.eye(4)
        tool[:3, :3] = s["tcp_rotation_world"]
        tool[:3, 3] = s["tcp"]
        goal = native @ relation
        report.append(
            {
                "attempt": i,
                "sensor_time_s": t,
                "nearest_native_time_s": r["sim_time_s"],
                "time_difference_s": abs(r["sim_time_s"] - t),
                "vision_vs_native_position_m": float(np.linalg.norm(vision[:3, 3] - native[:3, 3])),
                "vision_vs_native_angle_rad": float(
                    np.linalg.norm(rotation_error(vision[:3, :3], native[:3, :3]))
                ),
                "tool_vs_independent_native_goal_position_m": float(
                    np.linalg.norm(tool[:3, 3] - goal[:3, 3])
                ),
                "tool_vs_independent_native_goal_angle_rad": float(
                    np.linalg.norm(rotation_error(tool[:3, :3], goal[:3, :3]))
                ),
                "joint_error_rad": (np.array(s["q_rad"]) - s["command_q_rad"]).tolist(),
            }
        )
    result = {
        "scope": "Post-trial independent evaluation only. Simulator connector poses are never control targets. Nearest stored time used, with mismatch explicitly recorded. No success override.",
        "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "rows": report,
        "input_sha256": {
            str(p): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [
                root / "trajectory.json.gz",
                root / "validation.json",
                *observations,
                root / "primary_observation/detections.json",
            ]
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
