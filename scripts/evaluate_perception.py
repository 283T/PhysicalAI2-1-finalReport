"""Independent evaluator: ground-truth poses are never provided to the detector."""

import argparse
import json
import subprocess
from pathlib import Path

import numpy as np
from arm_kinematics import quaternion_rotation as rotation
from arm_kinematics import rotation_error
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
from processes import stop_process
from project_paths import ROOT


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--require-6d", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, default=ROOT / "configs/infeed_camera.json")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for name in ("validation.json", "evaluation_truth.json"):
        (args.output / name).unlink(missing_ok=True)
    transport = Node()
    samples = []

    def receive(msg):
        entries = {
            p.name: {
                "xyz": [p.position.x, p.position.y, p.position.z],
                "q": [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w],
            }
            for p in msg.pose
            if p.name in ("wire_harness_multi", "P1", "P2", "P3")
        }
        if len(entries) == 4:
            samples.append(
                {"stamp_ns": msg.header.stamp.sec * 10**9 + msg.header.stamp.nsec, "poses": entries}
            )
            if len(samples) > 2000:
                del samples[:1000]

    transport.subscribe(Pose_V, "/world/multi_connector_factory/dynamic_pose/info", receive)
    process = subprocess.Popen(
        [
            "python3",
            str(ROOT / "scripts/observe_connectors.py"),
            "--bridge",
            "--output",
            str(args.output),
            "--calibration",
            str(args.calibration),
            *(["--require-6d"] if args.require_6d else []),
        ],
        start_new_session=True,
    )
    try:
        process.wait(timeout=105)
        if process.returncode:
            raise RuntimeError("Perception run failed")
        detections = json.loads((args.output / "detections.json").read_text())
        if not samples:
            raise RuntimeError("No independent ground truth received")
        snapshot = min(
            samples, key=lambda sample: abs(sample["stamp_ns"] - detections["sensor_stamp_ns"])
        )
        skew = abs(snapshot["stamp_ns"] - detections["sensor_stamp_ns"]) / 1e9
        if skew > 0.1:
            raise RuntimeError("Ground truth is not synchronized to the sensor image")
        poses = snapshot["poses"]
        model = poses["wire_harness_multi"]
        model_rotation = rotation(model["q"])
        rows = []
        for detection in detections["connectors"]:
            plug = poses[detection["plug_id"]]
            center = np.asarray(model["xyz"]) + model_rotation @ np.asarray(plug["xyz"])
            orientation = model_rotation @ rotation(plug["q"])
            direction = orientation[:, 0]
            yaw = float(np.arctan2(direction[1], direction[0]))
            # Compare XY to the housing center; perceived Z is the visible top face, not center.
            xy_error = float(
                np.linalg.norm(center[:2] - np.asarray(detection["surface_xyz_m"])[:2])
            )
            angle_error = abs(
                float(
                    np.arctan2(
                        np.sin(detection["yaw_rad"] - yaw), np.cos(detection["yaw_rad"] - yaw)
                    )
                )
            )
            rows.append(
                {
                    "plug_id": detection["plug_id"],
                    "true_center_xyz_m": center.tolist(),
                    "xy_error_m": xy_error,
                    "yaw_error_rad": angle_error,
                    "surface_above_center_m": detection["surface_xyz_m"][2] - center[2],
                }
            )
            if "pose_6d" in detection:
                pose = detection["pose_6d"]
                rows[-1]["center_error_m"] = float(
                    np.linalg.norm(np.asarray(pose["center_xyz_m"]) - center)
                )
                rows[-1]["orientation_error_rad"] = float(
                    np.linalg.norm(
                        rotation_error(
                            np.asarray(pose["rotation_world_from_connector"]), orientation
                        )
                    )
                )
        result = {
            "success": len(rows) == 3
            and all(row["xy_error_m"] < 0.008 and row["yaw_error_rad"] < 0.15 for row in rows),
            "stamp_skew_s": skew,
            "connectors": rows,
            "scope": "RGB-D XY and directed planar yaw; ideal extrinsics, colored marked prototypes; no 6D pose or robot grasp",
        }
        if args.require_6d:
            result["success"] = len(rows) == 3 and all(
                row.get("center_error_m", float("inf")) < 0.005
                and row.get("orientation_error_rad", float("inf")) < 0.12
                for row in rows
            )
            result["scope"] = (
                "Full coded-prototype RGB-D center/orientation with ideal extrinsics; no object grasp or cable control"
            )
        (args.output / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
        (args.output / "evaluation_truth.json").write_text(json.dumps(snapshot, indent=2) + "\n")
        print(json.dumps(result, indent=2))
        if not result["success"]:
            raise RuntimeError("Perception accuracy validation failed")
    finally:
        # Release the native subscriber before Python finalizes the callback runtime.
        transport.unsubscribe("/world/multi_connector_factory/dynamic_pose/info")
        del transport
        stop_process(process, group=True)


if __name__ == "__main__":
    main()
