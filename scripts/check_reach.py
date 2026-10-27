"""Replay endpoint IK in isolated single-arm worlds; never move near the real harness."""

import argparse
import copy
import json
import os
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from arm_kinematics import quaternion_rotation as rotation
from arm_kinematics import rotation_error
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
from plan_reach import load_arms
from processes import stop_process
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def verify_arm(name, arm, source_world, goals, output, *, tolerances=None):
    gates = {"position_m": 0.006, "angle_rad": 0.04, "joint_rad": 0.02, "fk_m": 0.001}
    if tolerances is not None:
        if set(tolerances) != set(gates) or any(
            not np.isfinite(v) or v <= 0 for v in tolerances.values()
        ):
            raise ValueError("Finite positive position, angle, joint and FK tolerances required")
        gates = dict(tolerances)
    tree = ET.parse(source_world)
    world = tree.getroot().find("world")
    world.set("name", "reach_validation")
    model = copy.deepcopy(world.find(f"model[@name='{name}']"))
    # Single-arm motion validates kinematics/control, without asserting collision safety.
    for item in list(world):
        if item.tag == "model" or (
            item.tag == "plugin" and item.get("name") == "gz::sim::systems::Sensors"
        ):
            world.remove(item)
    world.append(model)
    scene = output / f"{name}.sdf"
    tree.write(scene, encoding="utf-8", xml_declaration=True)
    env = os.environ.copy()
    env["GZ_PARTITION"] = f"reach-check-{name}-{os.getpid()}"
    os.environ["GZ_PARTITION"] = env["GZ_PARTITION"]
    transport = Node()
    truth = []

    def receive(msg):
        for pose in msg.pose:
            if pose.name == name + "_rv4frl_hand_flange":
                matrix = rotation(
                    [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                )
                point = (
                    np.array([pose.position.x, pose.position.y, pose.position.z])
                    + matrix @ arm.tool[:3, 3]
                )
                truth[:] = [
                    (
                        time.monotonic(),
                        arm.base[:3, :3] @ point + arm.base[:3, 3],
                        arm.base[:3, :3] @ matrix,
                    )
                ]

    topic = "/world/reach_validation/dynamic_pose/info"
    transport.subscribe(Pose_V, topic, receive)
    node = rclpy.create_node("reach_check_" + name)
    state = []

    def receive_joints(msg):
        positions = dict(zip(msg.name, msg.position))
        if all(joint in positions for joint in arm.names):
            state[:] = [(time.monotonic(), np.array([positions[joint] for joint in arm.names]))]

    node.create_subscription(JointState, f"/{name}/joints", receive_joints, qos_profile_sensor_data)
    publishers = [node.create_publisher(Float64, f"/{name}/joint_{i}/cmd", 10) for i in range(6)]
    bridge = [
        "ros2",
        "run",
        "ros_gz_bridge",
        "parameter_bridge",
        f"/{name}/joints@sensor_msgs/msg/JointState[gz.msgs.Model",
    ]
    bridge += [f"/{name}/joint_{i}/cmd@std_msgs/msg/Float64]gz.msgs.Double" for i in range(6)]
    processes, results = [], []
    with (output / f"{name}.log").open("w") as log:
        try:
            for command in (["gz", "sim", "-s", "-r", str(scene)], bridge):
                processes.append(
                    subprocess.Popen(
                        command, env=env, stdout=log, stderr=log, start_new_session=True
                    )
                )
            end = time.monotonic() + 30
            while not (state and truth and all(p.get_subscription_count() for p in publishers)):
                if time.monotonic() > end:
                    raise RuntimeError("Missing joint feedback or independent flange pose")
                rclpy.spin_once(node, timeout_sec=0.05)
            for goal in goals:
                target = np.array(goal["target_pose"])
                q = np.array(goal["q_rad"])
                initial = state[0][1].copy()
                duration = max(3.0, float(np.max(np.abs(q - initial)) / 0.5))
                start = time.monotonic()
                consecutive = 0
                position_error = angle_error = joint_error = fk_error = float("inf")
                while time.monotonic() - start < duration + 45:
                    t = min(1.0, (time.monotonic() - start) / duration)
                    blend = t * t * (3 - 2 * t)
                    command = initial + (q - initial) * blend
                    for pub, value in zip(publishers, command):
                        pub.publish(Float64(data=float(value)))
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if (
                        t < 1
                        or time.monotonic() - truth[0][0] > 0.5
                        or time.monotonic() - state[0][0] > 0.5
                    ):
                        continue
                    position_error = float(np.linalg.norm(target[:3, 3] - truth[0][1]))
                    angle_error = float(np.linalg.norm(rotation_error(target[:3, :3], truth[0][2])))
                    joint_error = float(np.max(np.abs(q - state[0][1])))
                    fk = arm.forward(state[0][1])
                    fk_error = float(np.linalg.norm(fk[:3, 3] - truth[0][1]))
                    consecutive = (
                        consecutive + 1
                        if position_error < gates["position_m"]
                        and angle_error < gates["angle_rad"]
                        and joint_error < gates["joint_rad"]
                        and fk_error < gates["fk_m"]
                        else 0
                    )
                    if consecutive >= 5:
                        break
                row = {
                    **goal,
                    "arm": name,
                    "position_error_m": position_error,
                    "angle_error_rad": angle_error,
                    "joint_error_rad": joint_error,
                    "fk_ground_truth_error_m": fk_error,
                    "measured_tcp_pose": [
                        [*truth[0][2][i].tolist(), float(truth[0][1][i])] for i in range(3)
                    ] + [[0, 0, 0, 1]],
                    "acceptance_tolerances": gates,
                    "success": consecutive >= 5,
                }
                results.append(row)
                (output / "latest.json").write_text(json.dumps(results, indent=2) + "\n")
                print(
                    f"{name} {goal['plug_id']} {goal['stage']}: {row['success']} xyz={position_error:.5f} m",
                    flush=True,
                )
                if not row["success"]:
                    raise RuntimeError("Hover target feedback did not converge")
        finally:
            transport.unsubscribe(topic)
            del transport
            for process in reversed(processes):
                stop_process(process, group=True)
            node.destroy_node()
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--models", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    plan = json.loads(args.plan.read_text())
    if not plan["success"]:
        raise ValueError("Incomplete endpoint plan")
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "validation.json").unlink(missing_ok=True)
    arms = load_arms(args.world, args.models)
    # Isolate ROS feedback from the desktop, with separate Gazebo partitions per arm.
    os.environ["ROS_DOMAIN_ID"] = "173"
    rclpy.init()
    results = []
    try:
        for name, arm in arms.items():
            goals = []
            for connector in plan["connectors"]:
                option = connector["arms"][name]
                if not all(result["success"] for result in option.values()):
                    continue
                stages = (
                    ("pickup_hover", "pickup_grasp", "socket_hover")
                    if connector.get("grasp_candidate")
                    else ("pickup_hover", "socket_hover")
                )
                for stage in stages:
                    goals.append(
                        {
                            "plug_id": connector["plug_id"],
                            "stage": stage,
                            "q_rad": option[stage]["q_rad"],
                            "target_pose": (
                                connector["grasp_candidate"]["grasp_tcp_pose"]
                                if stage == "pickup_grasp"
                                else connector[stage + "_pose"]
                            ),
                        }
                    )
            if goals:
                results.extend(verify_arm(name, arm, args.world, goals, args.output))
        report = {
            "success": bool(results) and all(row["success"] for row in results),
            "targets": results,
            "scope": "Isolated single-arm replay of camera/drawing hover targets with independent Gazebo flange poses; no dual-arm collision checking, harness grasp or insertion",
        }
        (args.output / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
