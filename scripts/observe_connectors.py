"""ROS 2 RGB-D perception node. Publishes all three stable detections in world frame."""

import argparse
import json
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from connector_vision import detect_connectors
from perception_stream import StableConnectors, SyncedFrames, image_array
from processes import stop_process
from project_paths import ROOT
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/perception")
    parser.add_argument("--timeout", type=float, default=90)
    parser.add_argument(
        "--bridge", action="store_true", help="Start ROS-Gazebo camera bridge for this run"
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    # Never leave old detections available after a failed new acquisition.
    (args.output / "detections.json").unlink(missing_ok=True)
    calibration = json.loads((ROOT / "configs/infeed_camera.json").read_text())
    rclpy.init()
    node = rclpy.create_node("harness_connector_vision")
    frames = SyncedFrames()
    stability = StableConnectors()
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
    end = time.monotonic() + args.timeout
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
            detections, debug = detect_connectors(bgr, depth, info[-1].k, calibration)
            cv2.imwrite(str(args.output / "camera.png"), bgr)
            cv2.imwrite(str(args.output / "detections.png"), debug)
            np.save(args.output / "depth.npy", depth)
            (args.output / "latest.json").write_text(json.dumps(detections, indent=2) + "\n")
            if not stability.update(detections):
                continue
            result = {
                "source": "registered_rgbd_color_and_marker",
                "frame": calibration["frame"],
                "sensor_stamp_ns": current,
                "stable_frames": stability.count,
                "connectors": detections,
                "scope": "Visible prototype housings; top surface and planar yaw, not full 6D pose",
            }
            publisher.publish(String(data=json.dumps(result)))
            publish_end = time.monotonic() + 0.3
            while time.monotonic() < publish_end:
                rclpy.spin_once(node, timeout_sec=0.05)
            (args.output / "detections.json").write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps(result, indent=2), flush=True)
            return
        raise RuntimeError("No complete stable three-connector detection within timeout")
    finally:
        if bridge:
            stop_process(bridge, group=True)
        if bridge_log:
            bridge_log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
