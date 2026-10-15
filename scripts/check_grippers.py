"""Check both parallel grippers' open/close feedback without a held object."""

import argparse
import json
import os
import subprocess
import time
from pathlib import Path

import rclpy
from gripper_model import OPEN_TRAVEL_M, gap_m, measured_rest_gap
from processes import stop_process
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--models", type=Path, help="URDF directory; defaults to world directory")
    args = parser.parse_args()
    models = args.models or args.world.parent
    gaps = {
        name: measured_rest_gap(models / (name + ".urdf"), name + "_")
        for name in ("left_arm", "right_arm")
    }
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "validation.json").unlink(missing_ok=True)
    os.environ["ROS_DOMAIN_ID"] = "174"
    env = os.environ.copy()
    env["GZ_PARTITION"] = f"grippers-{os.getpid()}"
    rclpy.init()
    node = rclpy.create_node("gripper_check")
    states, publishers, processes = {}, {}, []
    bridge = ["ros2", "run", "ros_gz_bridge", "parameter_bridge"]
    for arm in ("left_arm", "right_arm"):

        def receive(msg, name=arm):
            states[name] = (time.monotonic(), dict(zip(msg.name, msg.position)))

        node.create_subscription(JointState, f"/{arm}/joints", receive, qos_profile_sensor_data)
        bridge.append(f"/{arm}/joints@sensor_msgs/msg/JointState[gz.msgs.Model")
        for finger in ("left", "right"):
            topic = f"/{arm}/gripper/{finger}/cmd"
            publishers[arm, finger] = node.create_publisher(Float64, topic, 10)
            bridge.append(topic + "@std_msgs/msg/Float64]gz.msgs.Double")
    results = []
    with (args.output / "grippers.log").open("w") as log:
        try:
            for command in (["gz", "sim", "-s", "-r", str(args.world)], bridge):
                processes.append(
                    subprocess.Popen(
                        command, env=env, stdout=log, stderr=log, start_new_session=True
                    )
                )
            deadline = time.monotonic() + 30
            while len(states) != 2 or not all(
                pub.get_subscription_count() for pub in publishers.values()
            ):
                if time.monotonic() > deadline:
                    raise RuntimeError("No gripper joint feedback")
                rclpy.spin_once(node, timeout_sec=0.05)
            for stage, goal in (("open", OPEN_TRAVEL_M), ("close", 0.0), ("reopen", OPEN_TRAVEL_M)):
                deadline = time.monotonic() + 15
                stable = 0
                errors = {}
                while time.monotonic() < deadline:
                    for pub in publishers.values():
                        pub.publish(Float64(data=goal))
                    rclpy.spin_once(node, timeout_sec=0.05)
                    errors = {
                        f"{arm}_{finger}": abs(
                            states[arm][1].get(arm + "_finger_" + finger + "_joint", float("inf"))
                            - goal
                        )
                        for arm, finger in publishers
                    }
                    fresh = all(time.monotonic() - row[0] < 0.5 for row in states.values())
                    stable = stable + 1 if fresh and max(errors.values()) < 0.0005 else 0
                    if stable >= 5:
                        break
                row = {
                    "stage": stage,
                    "target_travel_m": goal,
                    "target_gap_m": {arm: gap_m(goal, gap) for arm, gap in gaps.items()},
                    "errors_m": errors,
                    "success": stable >= 5,
                }
                results.append(row)
                print(json.dumps(row), flush=True)
                if not row["success"]:
                    raise RuntimeError("Gripper position did not converge")
            report = {
                "success": True,
                "stages": results,
                "scope": "Both grippers opening/closing empty; no contact-force or object holding validation",
            }
            (args.output / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
        finally:
            for process in reversed(processes):
                stop_process(process, group=True)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
