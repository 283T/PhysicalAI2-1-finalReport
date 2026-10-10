"""Contact-only cable grip benchmark; Cartesian test fixture, not a MELFA motion."""

import argparse
import json
import math
import os
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from build_case import harness_model
from build_world import box, child, link, plugin
from grasp_validation import assess_hold, world_link_positions
from gripper_model import OPEN_TRAVEL_M, add_controllers, add_gripper
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
from processes import stop_process
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def build_benchmark(output, friction=1.0):
    """Reuse production fingers on a vertical test carriage, with an unanchored cable."""
    output.mkdir(parents=True, exist_ok=True)
    robot = ET.Element("robot", name="grip_fixture")
    child(robot, "link", name="world")
    carriage = child(robot, "link", name="carriage")
    inertial = child(carriage, "inertial")
    child(inertial, "mass", value=".1")
    child(inertial, "inertia", ixx=".0001", iyy=".0001", izz=".0001", ixy="0", ixz="0", iyz="0")
    lift = child(robot, "joint", name="lift", type="prismatic")
    child(lift, "parent", link="world")
    child(lift, "child", link="carriage")
    child(lift, "origin", xyz="0 0 .35", rpy=f"{math.pi} 0 {math.pi / 2}")
    child(lift, "axis", xyz="0 0 -1")
    child(lift, "limit", lower="-.15", upper=".15", effort="100", velocity=".03")
    add_gripper(robot, "bench_", "carriage")
    urdf = output / "gripper.urdf"
    ET.ElementTree(robot).write(urdf, encoding="utf-8", xml_declaration=True)
    converted = subprocess.run(
        ["gz", "sdf", "-p", str(urdf)], capture_output=True, text=True, check=True
    )
    model = ET.fromstring(converted.stdout).find("model")
    for collision in model.findall(".//collision"):
        surface = child(collision, "surface")
        ode = child(child(surface, "friction"), "ode")
        child(ode, "mu", friction)
        child(ode, "mu2", friction)
    add_controllers(model, "bench_", "/grip_bench/gripper")
    plugin(
        model,
        ("joint-position-controller", "JointPositionController"),
        joint_name="lift",
        topic="/grip_bench/lift/cmd",
        p_gain=500,
        d_gain=20,
        i_gain=100,
        i_max=100,
        i_min=-100,
        cmd_max=100,
        cmd_min=-100,
    )
    plugin(model, ("joint-state-publisher", "JointStatePublisher"), topic="/grip_bench/joints")
    plugin(
        model,
        ("pose-publisher", "PosePublisher"),
        publish_link_pose="true",
        publish_model_pose="false",
        use_pose_vector_msg="true",
        update_frequency=30,
        topic="/grip_bench/poses",
    )
    sdf = ET.Element("sdf", version="1.10")
    world = child(sdf, "world", name="grip_contact_benchmark")
    child(world, "gravity", "0 0 -9.81")
    physics = child(world, "physics", name="physics", type="ignored")
    child(physics, "max_step_size", 0.001)
    child(physics, "real_time_factor", 1)
    plugin(world, ("physics", "Physics"))
    plugin(world, ("user-commands", "UserCommands"))
    plugin(world, ("scene-broadcaster", "SceneBroadcaster"))
    platform = child(world, "model", name="platform")
    child(platform, "static", "true")
    box(link(platform, "base"), "support", ".8 .5 .04", "-.15 0 .12 0 0 0", ".3 .3 .3 1")
    world.append(model)
    harness = harness_model(
        world,
        {
            "pose_m_rad": [-0.32, 0, 0.15, 0, 0, 0],
            "branches": [
                {
                    "plug_id": "P1",
                    "segments": 12,
                    "angle_rad": 0,
                    "curve_rad": 0,
                    "color": ".8 .08 .04 1",
                }
            ],
        },
    )
    # Pin contact/insertion is excluded: this benchmark tests housing friction only.
    terminal = harness.find("link[@name='P1']")
    for item in list(terminal):
        if item.get("name", "").startswith("pin_"):
            terminal.remove(item)
    ET.indent(sdf)
    path = output / "world.sdf"
    ET.ElementTree(sdf).write(path, encoding="utf-8", xml_declaration=True)
    return path


def run_trial(output, friction):
    scene = build_benchmark(output, friction)
    os.environ["GZ_PARTITION"] = f"grip-contact-{os.getpid()}-{friction}"
    transport = Node()
    harness, fixture, states = {}, {}, {}
    joint_stamp = [0.0]

    def receive(msg):
        poses = {
            p.name: {
                "xyz": [p.position.x, p.position.y, p.position.z],
                "q": [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w],
            }
            for p in msg.pose
        }
        if not all(
            name in poses for name in ("wire_harness_multi", "grip_fixture", "P1", "carriage")
        ):
            return
        stamp = time.monotonic()
        cable_names = [name for name in poses if name.startswith("P1_wire_")] + ["P1", "junction"]
        cable = world_link_positions(poses, "wire_harness_multi", cable_names)
        tcp = world_link_positions(poses, "grip_fixture", ["carriage"])
        harness.update({name: (stamp, point) for name, point in cable.items()})
        fixture.update({name: (stamp, point) for name, point in tcp.items()})

    pose_topic = "/world/grip_contact_benchmark/dynamic_pose/info"
    transport.subscribe(Pose_V, pose_topic, receive)
    node = rclpy.create_node("grip_contact_benchmark")

    def joints(msg):
        states.clear()
        states.update(dict(zip(msg.name, msg.position)))
        joint_stamp[0] = time.monotonic()

    node.create_subscription(JointState, "/grip_bench/joints", joints, qos_profile_sensor_data)
    topics = [
        "/grip_bench/lift/cmd",
        "/grip_bench/gripper/left/cmd",
        "/grip_bench/gripper/right/cmd",
    ]
    pubs = [node.create_publisher(Float64, topic, 10) for topic in topics]
    bridge = [
        "ros2",
        "run",
        "ros_gz_bridge",
        "parameter_bridge",
        "/grip_bench/joints@sensor_msgs/msg/JointState[gz.msgs.Model",
    ]
    bridge += [topic + "@std_msgs/msg/Float64]gz.msgs.Double" for topic in topics]
    processes, history = [], []
    report = {
        "success": False,
        "friction_mu": friction,
        "scope": "Cartesian contact fixture, one pinless terminal and 12-segment cable; no arm, vision, dual-arm support or insertion",
    }

    def sample():
        if "P1" not in harness or "carriage" not in fixture or len(states) != 3:
            return None
        if min(harness["P1"][0], fixture["carriage"][0], joint_stamp[0]) < time.monotonic() - 0.5:
            return None
        return {
            "time_s": time.monotonic(),
            "plug": harness["P1"][1].tolist(),
            "tcp": (fixture["carriage"][1] + [0, 0, -0.08]).tolist(),
            "fingers": [states["bench_finger_" + f + "_joint"] for f in ("left", "right")],
            "cable_z_span_m": float(np.ptp([p[1][2] for p in list(harness.values())])),
        }

    def phase(name, lift, grip, duration, initial=None):
        start = time.monotonic()
        rows, last_stamp = [], None
        while time.monotonic() - start < duration:
            t = min(1, (time.monotonic() - start) / max(1, duration - 2))
            command = (
                lift
                if initial is None
                else initial + (lift - initial) * (3 * t * t - 2 * t * t * t)
            )
            for pub, value in zip(pubs, (command, grip, grip)):
                pub.publish(Float64(data=float(value)))
            rclpy.spin_once(node, timeout_sec=0.03)
            row = sample()
            stamp = harness.get("P1", (None,))[0]
            if row is not None and stamp != last_stamp:
                row.update(phase=name, elapsed_s=time.monotonic() - start)
                rows.append(row)
                last_stamp = stamp
        if not rows:
            raise RuntimeError("No fresh physics observations")
        history.extend(rows)
        print(
            f"{name}: plug_z={rows[-1]['plug'][2]:.4f} tcp_z={rows[-1]['tcp'][2]:.4f}", flush=True
        )
        return rows

    with (output / "simulation.log").open("w") as log:
        try:
            for cmd in (["gz", "sim", "-s", "-r", str(scene)], bridge):
                processes.append(
                    subprocess.Popen(cmd, stdout=log, stderr=log, start_new_session=True)
                )
            end = time.monotonic() + 30
            while sample() is None or not all(pub.get_subscription_count() for pub in pubs):
                if time.monotonic() > end:
                    raise RuntimeError("Missing gripper or harness feedback")
                rclpy.spin_once(node, timeout_sec=0.05)
            baseline = phase("open", 0, OPEN_TRAVEL_M, 5)[-1]
            phase("approach", -0.118, OPEN_TRAVEL_M, 8, initial=0)
            closed = phase("close", -0.118, 0, 6)[-1]
            phase("lift", -0.018, 0, 8, initial=-0.118)
            held = phase("hold", -0.018, 0, 4)
            hold_result = assess_hold(baseline, held, closed)
            released = phase("release", -0.018, OPEN_TRAVEL_M, 5)
            fall = float(held[-1]["plug"][2] - released[-1]["plug"][2])
            report.update(
                hold=hold_result,
                release_drop_m=fall,
                release_success=fall > 0.03,
                baseline=baseline,
                final=released[-1],
                success=hold_result["success"] and fall > 0.03,
            )
        except Exception as error:
            report["error"] = str(error)
        finally:
            transport.unsubscribe(pose_topic)
            for process in reversed(processes):
                stop_process(process, group=True)
            node.destroy_node()
            (output / "trajectory.json").write_text(json.dumps(history, indent=2) + "\n")
            (output / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--friction", type=float, default=1.0)
    parser.add_argument(
        "--expect-slip",
        action="store_true",
        help="Negative control: require completed lifting with failed holding",
    )
    args = parser.parse_args()
    if not math.isfinite(args.friction) or args.friction < 0:
        raise ValueError("Friction must be finite and nonnegative")
    args.output.mkdir(parents=True, exist_ok=True)
    for name in ("validation.json", "trajectory.json"):
        (args.output / name).unlink(missing_ok=True)
    os.environ["ROS_DOMAIN_ID"] = "175"
    rclpy.init()
    try:
        result = run_trial(args.output, args.friction)
        expectation_met = (
            "error" not in result
            and not result["hold"]["success"]
            and result["hold"]["minimum_tcp_lift_m"] > 0.07
            if args.expect_slip
            else result["success"]
        )
        result.update(
            expectation="slip" if args.expect_slip else "hold_and_release",
            expectation_met=expectation_met,
        )
        (args.output / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
        if not expectation_met:
            raise RuntimeError("Contact grip benchmark failed; see validation.json")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
