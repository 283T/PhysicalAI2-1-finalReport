"""Try one RGB-D-derived grip/lift/release with MELFA in the running factory scene."""

import argparse
import hashlib
import json
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from arm_kinematics import quaternion_rotation, rotation_error
from cartesian_path import interpolate_joints, translation_path
from grasp_validation import assess_hold, world_link_positions
from gripper_model import OPEN_TRAVEL_M
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
from plan_reach import load_arms, plan_reach
from processes import stop_process
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--board", type=Path, required=True)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--models", type=Path, default=Path("outputs/melfa_worlds"))
    parser.add_argument("--plug", choices=("P1", "P2", "P3"), default="P1")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--keep-camera",
        action="store_true",
        help="Keep full RGB-D rendering rate after the accepted static observation",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "validation.json").unlink(missing_ok=True)
    observations = args.output / "observations"
    # Capture from the currently running scene; never replay a previous RGB-D snapshot.
    subprocess.run(
        [
            "python3",
            "scripts/observe_connectors.py",
            "--bridge",
            "--require-6d",
            "--output",
            str(observations),
        ],
        check=True,
    )
    detections = json.loads((observations / "detections.json").read_text())
    arms = load_arms(args.world, args.models)
    plan = plan_reach(
        json.loads(args.plan.read_text()), json.loads(args.board.read_text()), detections, arms
    )
    (args.output / "grasp_plan.json").write_text(json.dumps(plan, indent=2) + "\n")
    row = next(r for r in plan["connectors"] if r["plug_id"] == args.plug)
    if not row["selected_arm"] or not row["grasp_candidate"]:
        raise ValueError("Selected terminal has no complete 6D grasp/hover solution")
    name = row["selected_arm"]
    arm = arms[name]
    candidate = row["grasp_candidate"]
    hover = np.asarray(candidate["pickup_hover_pose"])
    grip = np.asarray(candidate["grasp_tcp_pose"])
    lift = grip.copy()
    lift[2, 3] += 0.10
    report = {
        "success": False,
        "plug_id": args.plug,
        "arm": name,
        "scope": "One RGB-D-derived MELFA pickup attempt in the full scene; no collision planning, second-arm support or insertion",
        "world_sha256": hashlib.sha256(args.world.read_bytes()).hexdigest(),
    }
    sensor = ET.parse(args.world).getroot().find("world/model[@name='overhead_camera']/link/sensor")
    camera_service = sensor.findtext("topic") + "/set_rate"
    original_rate = float(sensor.findtext("update_rate"))
    throttled = False

    def set_camera_rate(rate):
        subprocess.run(
            [
                "gz",
                "service",
                "-s",
                camera_service,
                "--reqtype",
                "gz.msgs.Double",
                "--req",
                f"data: {rate}",
                "--timeout",
                "2000",
            ],
            check=True,
            capture_output=True,
            timeout=5,
        )

    rclpy.init()
    node = rclpy.create_node("melfa_pick_connector")
    transport = Node()
    truth, joints = [], []
    flange = name + "_rv4frl_hand_flange"
    pose_topic = "/world/multi_connector_factory/dynamic_pose/info"

    def receive(msg):
        poses = {
            p.name: {
                "xyz": [p.position.x, p.position.y, p.position.z],
                "q": [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w],
            }
            for p in msg.pose
        }
        if not all(key in poses for key in (name, flange, "wire_harness_multi", args.plug)):
            return
        model_rotation = quaternion_rotation(poses[name]["q"])
        tool_rotation = model_rotation @ quaternion_rotation(poses[flange]["q"])
        tool_position = (
            world_link_positions(poses, name, [flange])[flange] + tool_rotation @ arm.tool[:3, 3]
        )
        plugs = world_link_positions(poses, "wire_harness_multi", ["P1", "P2", "P3"])
        cable_names = [key for key in poses if "_wire_" in key]
        cable = world_link_positions(poses, "wire_harness_multi", cable_names)
        sim_time = msg.header.stamp.sec + msg.header.stamp.nsec / 1e9
        truth[:] = [(time.monotonic(), tool_position, tool_rotation, plugs, cable, sim_time)]

    transport.subscribe(Pose_V, pose_topic, receive)

    def receive_joints(msg):
        positions = dict(zip(msg.name, msg.position))
        if all(key in positions for key in arm.names):
            joints[:] = [(time.monotonic(), positions)]

    node.create_subscription(JointState, f"/{name}/joints", receive_joints, qos_profile_sensor_data)
    topics = [f"/{name}/joint_{i}/cmd" for i in range(6)] + [
        f"/{name}/gripper/{f}/cmd" for f in ("left", "right")
    ]
    pubs = [node.create_publisher(Float64, topic, 10) for topic in topics]
    bridge_command = [
        "ros2",
        "run",
        "ros_gz_bridge",
        "parameter_bridge",
        f"/{name}/joints@sensor_msgs/msg/JointState[gz.msgs.Model",
    ]
    bridge_command += [topic + "@std_msgs/msg/Float64]gz.msgs.Double" for topic in topics]
    bridge = None
    history = []
    last_saved = [-1.0, -1.0]
    target_q = None
    target_grip = OPEN_TRAVEL_M

    def sample():
        if not truth or not joints or min(truth[0][0], joints[0][0]) < time.monotonic() - 0.5:
            return None
        positions = joints[0][1]
        finger_names = [name + "_finger_" + f + "_joint" for f in ("left", "right")]
        if not all(key in positions for key in finger_names):
            return None
        return {
            "time_s": time.monotonic(),
            "sim_time_s": truth[0][5],
            "plug": truth[0][3][args.plug].tolist(),
            "tcp": truth[0][1].tolist(),
            "fingers": [positions[key] for key in finger_names],
            "q_rad": [positions[key] for key in arm.names],
            "cable_z_span_m": float(np.ptp([point[2] for point in truth[0][4].values()])),
        }

    def publish():
        for pub, value in zip(pubs, [*target_q, target_grip, target_grip]):
            pub.publish(Float64(data=float(value)))
        rclpy.spin_once(node, timeout_sec=0.03)

    def record(phase):
        row = sample()
        if row is not None:
            row["phase"] = phase
            row["command_q_rad"] = target_q.tolist()
            row["command_tcp_xyz_m"] = arm.forward(target_q)[:3, 3].tolist()
            if (
                not history
                or history[-1]["phase"] != phase
                or row["sim_time_s"] - last_saved[0] >= 0.02
            ):
                history.append(row)
                last_saved[0] = row["sim_time_s"]
            if row["time_s"] - last_saved[1] > 0.5:
                (args.output / "latest.json").write_text(json.dumps(row, indent=2) + "\n")
                last_saved[1] = row["time_s"]
        return row

    def dwell(phase, duration):
        print(f"{phase}: start ({duration:g} simulated seconds)", flush=True)
        rows, last = [], None
        start = truth[0][5]
        wall_start = time.monotonic()
        while truth[0][5] - start < duration:
            if time.monotonic() - wall_start > 30 + duration * 15:
                raise RuntimeError("Simulation stalled in " + phase)
            publish()
            stamp = truth[0][0] if truth else None
            if stamp != last:
                row = record(phase)
                if row:
                    rows.append(row)
                    last = stamp
        if len(rows) < 10:
            raise RuntimeError("Insufficient fresh observations in " + phase)
        return rows

    def move(phase, target, *, straight=False):
        nonlocal target_q
        print(f"{phase}: preflight and move", flush=True)
        current = np.asarray(sample()["q_rad"])
        if straight:
            path = translation_path(arm, current, target)
            duration = max(3, np.linalg.norm(target[:3, 3] - arm.forward(current)[:3, 3]) / 0.02)
        else:
            result = arm.inverse(target, seed=current)
            if not result["success"]:
                raise RuntimeError("Unreachable " + phase)
            path = np.asarray([current, result["q_rad"]])
            duration = max(3, np.max(np.abs(path[-1] - current)) / 0.5)
        start = truth[0][5]
        wall_start = time.monotonic()
        stable, last = 0, None
        error = angle = float("inf")
        while truth[0][5] - start < duration + 15 and time.monotonic() - wall_start < 180:
            t = min(1, (truth[0][5] - start) / duration)
            target_q = interpolate_joints(path, t * t * (3 - 2 * t))
            publish()
            observation = record(phase)
            if observation is None or t < 1:
                continue
            actual = arm.forward(observation["q_rad"])
            error = np.linalg.norm(target[:3, 3] - truth[0][1])
            angle = np.linalg.norm(rotation_error(target[:3, :3], truth[0][2]))
            valid = (
                error < 0.002
                and angle < 0.025
                and np.max(np.abs(path[-1] - observation["q_rad"])) < 0.01
            )
            valid = valid and np.linalg.norm(actual[:3, 3] - truth[0][1]) < 0.001
            if truth[0][0] != last:
                stable = stable + 1 if valid else 0
                last = truth[0][0]
            if stable >= 5:
                print(f"{phase}: TCP error {error * 1000:.2f} mm", flush=True)
                return
        raise RuntimeError(
            f"{phase} did not converge: position {error:.5f} m, angle {angle:.5f} rad"
        )

    with (args.output / "control.log").open("w") as log:
        try:
            if not args.keep_camera:
                set_camera_rate(0.01)
                throttled = True
                report["camera_rate_during_motion_hz"] = 0.01
            bridge = subprocess.Popen(
                bridge_command, stdout=log, stderr=log, start_new_session=True
            )
            end = time.monotonic() + 30
            while sample() is None or not all(pub.get_subscription_count() for pub in pubs):
                if time.monotonic() > end:
                    raise RuntimeError("Missing MELFA/gripper feedback")
                rclpy.spin_once(node, timeout_sec=0.05)
            target_q = np.asarray(sample()["q_rad"])
            measured_fk = arm.forward(target_q)
            if (
                np.linalg.norm(measured_fk[:3, 3] - truth[0][1]) > 0.001
                or np.linalg.norm(rotation_error(measured_fk[:3, :3], truth[0][2])) > 0.01
            ):
                raise RuntimeError("Current world and supplied arm kinematics disagree")
            baseline = dwell("open", 1.5)[-1]
            move("hover", hover)
            moved = np.linalg.norm(np.asarray(sample()["plug"]) - baseline["plug"])
            if moved > 0.004:
                raise RuntimeError("Terminal moved before descent; need new image")
            move("approach", grip, straight=True)
            target_grip = 0.0
            closed = dwell("close", 4)[-1]
            move("lift", lift, straight=True)
            held = dwell("hold", 4)
            hold = assess_hold(baseline, held, closed)
            target_grip = OPEN_TRAVEL_M
            released = dwell("release", 3)[-1]
            fall = held[-1]["plug"][2] - released["plug"][2]
            report.update(
                hold=hold,
                release_drop_m=fall,
                baseline=baseline,
                closed=closed,
                final=released,
                success=hold["success"] and fall > 0.03,
            )
        except Exception as error:
            report["error"] = str(error)
        finally:
            if target_q is not None:
                target_grip = OPEN_TRAVEL_M
                try:
                    dwell("final_open", 1)
                except RuntimeError:
                    pass
            transport.unsubscribe(pose_topic)
            if bridge:
                stop_process(bridge, group=True)
            if throttled:
                try:
                    set_camera_rate(original_rate)
                    report["camera_rate_restored_hz"] = original_rate
                except (subprocess.SubprocessError, OSError) as error:
                    report["camera_restore_error"] = str(error)
            node.destroy_node()
            rclpy.shutdown()
            (args.output / "trajectory.json").write_text(json.dumps(history, indent=2) + "\n")
            (args.output / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if not report["success"]:
        raise RuntimeError("MELFA pickup trial failed; see validation.json")


if __name__ == "__main__":
    main()
