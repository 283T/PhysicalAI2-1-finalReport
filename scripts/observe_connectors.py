"""ROS 2 RGB-D perception node. Publishes all three stable detections in world frame."""

import argparse
import json
import math
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from connector_vision import detect_connectors
from end_markers import detect_end_markers
from inspection_camera import validate_calibration
from perception_stream import StableConnectors, SyncedFrames, image_array
from processes import stop_process
from project_paths import ROOT
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/perception")
    parser.add_argument("--calibration", type=Path, default=ROOT / "configs/infeed_camera.json")
    parser.add_argument("--end-markers", action="store_true")
    parser.add_argument(
        "--plugs", choices=("P1", "P2", "P3"), nargs="+", default=["P1", "P2", "P3"]
    )
    parser.add_argument(
        "--require-6d", action="store_true", help="Require complete stable coded-face 6D poses"
    )
    parser.add_argument("--timeout", type=float, default=90)
    parser.add_argument(
        "--bridge", action="store_true", help="Start ROS-Gazebo camera bridge for this run"
    )
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or not 5 <= args.timeout <= 300:
        parser.error("--timeout must be finite and between 5 and 300 seconds")
    if args.end_markers:
        args.require_6d = True
    args.output.mkdir(parents=True, exist_ok=True)
    # Never leave old detections available after a failed new acquisition.
    (args.output / "detections.json").unlink(missing_ok=True)
    calibration = validate_calibration(json.loads(args.calibration.read_text()))
    (args.output / "calibration.json").write_text(json.dumps(calibration, indent=2) + "\n")
    rclpy.init()
    node = rclpy.create_node("harness_connector_vision")
    frames = SyncedFrames()
    stability = StableConnectors(require_6d=args.require_6d, required_ids=args.plugs)
    info = []
    prefix = calibration["topic"]
    node.create_subscription(
        Image, prefix + "/image", lambda msg: frames.add(msg, "rgb"), qos_profile_sensor_data
    )
    node.create_subscription(
        Image,
        prefix + "/depth_image",
        lambda msg: frames.add(msg, "depth"),
        qos_profile_sensor_data,
    )
    node.create_subscription(
        CameraInfo,
        prefix + "/camera_info",
        lambda msg: info.__setitem__(slice(None), [msg]),
        qos_profile_sensor_data,
    )
    publisher = node.create_publisher(String, "/factory/connector_detections", 10)
    started = time.monotonic()
    acquisition = {"processed_frames": 0, "frames": [], "success": False, "timeout_s": args.timeout}
    end = started + args.timeout
    bridge = None
    bridge_log = None
    try:
        if args.bridge:
            bridge_log = (args.output / "bridge.log").open("w")
            bridge = subprocess.Popen(
                [
                    "ros2",
                    "run",
                    "ros_gz_bridge",
                    "parameter_bridge",
                    prefix + "/image@sensor_msgs/msg/Image[gz.msgs.Image",
                    prefix + "/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                    prefix + "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                ],
                stdout=bridge_log,
                stderr=bridge_log,
                start_new_session=True,
            )
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if not info:
                continue
            pair = frames.take_latest()
            if pair is None:
                continue
            current, rgb_msg, depth_msg = pair
            if rgb_msg.header.frame_id != depth_msg.header.frame_id or (
                info[-1].width,
                info[-1].height,
            ) != (rgb_msg.width, rgb_msg.height):
                raise ValueError("Camera stream frames or CameraInfo resolution disagree")
            bgr, depth = image_array(rgb_msg), image_array(depth_msg)
            detector = detect_end_markers if args.end_markers else detect_connectors
            detections, debug = detector(bgr, depth, info[-1].k, calibration)
            detections = [row for row in detections if row["plug_id"] in args.plugs]
            cv2.imwrite(str(args.output / "camera.png"), bgr)
            cv2.imwrite(str(args.output / "detections.png"), debug)
            np.save(args.output / "depth.npy", depth)
            (args.output / "camera_intrinsics.json").write_text(json.dumps(list(info[-1].k)) + "\n")
            (args.output / "latest.json").write_text(json.dumps(detections, indent=2) + "\n")
            accepted = stability.update(detections)
            acquisition["processed_frames"] += 1
            acquisition["frames"].append(
                {
                    "sensor_stamp_ns": current,
                    "elapsed_wall_s": time.monotonic() - started,
                    "detected_plugs": [row["plug_id"] for row in detections],
                    "stable_frames": stability.count,
                }
            )
            acquisition["success"] = accepted
            (args.output / "acquisition.json").write_text(json.dumps(acquisition, indent=2) + "\n")
            if not accepted:
                continue
            result = {
                "source": "registered_rgbd_end_tags"
                if args.end_markers
                else "registered_rgbd_color_and_marker",
                "required_plugs": sorted(set(args.plugs)),
                "frame": calibration["frame"],
                "sensor_stamp_ns": current,
                "stable_frames": stability.count,
                "connectors": detections,
                "scope": (
                    "Coded prototype housings: full pose and candidate grasp; no holding validation"
                    if all("pose_6d" in row for row in detections)
                    else "Visible prototype housings; top surface and planar yaw, not full 6D pose"
                ),
            }
            publisher.publish(String(data=json.dumps(result)))
            publish_end = time.monotonic() + 0.3
            while time.monotonic() < publish_end:
                rclpy.spin_once(node, timeout_sec=0.05)
            (args.output / "detections.json").write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps(result, indent=2), flush=True)
            return
        raise RuntimeError("No complete stable detection of required plugs within timeout")
    except Exception as error:
        acquisition["error"] = {"type": type(error).__name__, "message": str(error)}
        raise
    finally:
        acquisition["elapsed_wall_s"] = time.monotonic() - started
        (args.output / "acquisition.json").write_text(json.dumps(acquisition, indent=2) + "\n")
        if bridge:
            stop_process(bridge, group=True)
        if bridge_log:
            bridge_log.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
