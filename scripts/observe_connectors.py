"""ROS 2 RGB-D perception node. Publishes all three stable detections in world frame."""
import argparse
import json
import subprocess
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from connector_vision import detect_connectors

ROOT = Path(__file__).resolve().parents[1]


def stamp(msg):
    return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec


def image_array(msg):
    endian = '>' if msg.is_bigendian else '<'
    if msg.encoding in ('rgb8', 'bgr8'):
        data = np.ndarray((msg.height, msg.width, 3), dtype='u1', buffer=bytes(msg.data),
                          strides=(msg.step, 3, 1)).copy()
        return cv2.cvtColor(data, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else data
    if msg.encoding == '32FC1':
        return np.ndarray((msg.height, msg.width), dtype=endian+'f4', buffer=bytes(msg.data),
                          strides=(msg.step, 4)).astype(np.float32)
    raise ValueError('Unsupported image encoding: '+msg.encoding)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=ROOT/'outputs/perception')
    parser.add_argument('--timeout', type=float, default=90)
    parser.add_argument('--bridge', action='store_true', help='Start ROS-Gazebo camera bridge for this run')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    # Never leave old detections available after a failed new acquisition.
    (args.output/'detections.json').unlink(missing_ok=True)
    calibration = json.loads((ROOT/'configs/infeed_camera.json').read_text())
    rclpy.init()
    node = rclpy.create_node('harness_connector_vision')
    images = {'rgb': {}, 'depth': {}}
    info = []
    def receive(msg, kind):
        images[kind][stamp(msg)] = msg
        while len(images[kind]) > 10:
            del images[kind][min(images[kind])]
    prefix = calibration['topic']
    node.create_subscription(Image, prefix+'/image', lambda msg: receive(msg, 'rgb'), qos_profile_sensor_data)
    node.create_subscription(Image, prefix+'/depth_image', lambda msg: receive(msg, 'depth'), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, prefix+'/camera_info', lambda msg: info.__setitem__(slice(None), [msg]), qos_profile_sensor_data)
    publisher = node.create_publisher(String, '/factory/connector_detections', 10)
    previous, previous_yaws, stable, last_stamp = None, None, 0, -1
    end = time.monotonic()+args.timeout
    bridge = None
    bridge_log = None
    try:
        if args.bridge:
            bridge_log = (args.output/'bridge.log').open('w')
            bridge = subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                prefix+'/image@sensor_msgs/msg/Image[gz.msgs.Image',
                prefix+'/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                prefix+'/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
                stdout=bridge_log, stderr=bridge_log)
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
            shared = set(images['rgb']) & set(images['depth'])
            if not shared or not info:
                continue
            current = max(shared)
            if current <= last_stamp:
                continue
            last_stamp = current
            rgb_msg, depth_msg = images['rgb'][current], images['depth'][current]
            if (rgb_msg.header.frame_id != depth_msg.header.frame_id or
                (info[-1].width, info[-1].height) != (rgb_msg.width, rgb_msg.height)):
                raise ValueError('Camera stream frames or CameraInfo resolution disagree')
            bgr, depth = image_array(rgb_msg), image_array(depth_msg)
            detections, debug = detect_connectors(bgr, depth, info[-1].k, calibration)
            cv2.imwrite(str(args.output/'camera.png'), bgr)
            cv2.imwrite(str(args.output/'detections.png'), debug)
            np.save(args.output/'depth.npy', depth)
            (args.output/'latest.json').write_text(json.dumps(detections, indent=2)+'\n')
            ids = {row['plug_id'] for row in detections}
            if ids != {'P1', 'P2', 'P3'}:
                stable, previous, previous_yaws = 0, None, None
                continue
            positions = np.array([row['surface_xyz_m'] for row in sorted(detections, key=lambda row: row['plug_id'])])
            yaws = np.array([row['yaw_rad'] for row in sorted(detections, key=lambda row: row['plug_id'])])
            yaw_change = np.max(np.abs(np.arctan2(np.sin(yaws-previous_yaws), np.cos(yaws-previous_yaws)))) if previous_yaws is not None else float('inf')
            stable = stable+1 if previous is not None and np.max(np.linalg.norm(positions-previous, axis=1)) < .003 and yaw_change < .08 else 1
            previous = positions
            previous_yaws = yaws
            if stable < 3:
                continue
            result = {'source': 'registered_rgbd_color_and_marker', 'frame': calibration['frame'],
                      'sensor_stamp_ns': current, 'stable_frames': stable, 'connectors': detections,
                      'scope': 'Visible prototype housings; top surface and planar yaw, not full 6D pose'}
            publisher.publish(String(data=json.dumps(result)))
            publish_end = time.monotonic()+.3
            while time.monotonic() < publish_end:
                rclpy.spin_once(node, timeout_sec=.05)
            (args.output/'detections.json').write_text(json.dumps(result, indent=2)+'\n')
            print(json.dumps(result, indent=2), flush=True)
            return
        raise RuntimeError('No complete stable three-connector detection within timeout')
    finally:
        if bridge:
            bridge.terminate()
            try:
                bridge.wait(timeout=5)
            except subprocess.TimeoutExpired:
                bridge.kill()
                bridge.wait()
        if bridge_log:
            bridge_log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
