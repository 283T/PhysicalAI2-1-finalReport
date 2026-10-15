"""Feedback, bounded motion and evidence for factory arm experiments.

Object poses are exposed only for independent evaluation. Joint feedback and
robot flange poses may be used for control. No object pose command is provided.
"""

import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from arm_collision_bounds import (
    ArmCollisionBounds,
    FingerStateBounds,
    finger_collision_intervals,
    screen_paths,
    with_parked_arms,
)
from arm_contacts import CONTACT_TOPIC, accumulate_contact, inter_arm_pair
from arm_kinematics import quaternion_rotation, rotation_error
from cartesian_path import interpolate_joints
from fixture_collision_bounds import FixtureBounds, sdf_pose
from gripper_model import OPEN_TRAVEL_M
from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
from plan_reach import load_arms
from processes import stop_process
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class ArmSession:
    def __init__(self, world, models, log):
        self.arms = load_arms(world, models)
        self.bounds = {
            n: ArmCollisionBounds(models / (n + ".urdf"), a) for n, a in self.arms.items()
        }
        self.fixtures = FixtureBounds(world, include_carrier=True)
        carrier = ET.parse(world).getroot().find("world/model[@name='conveyor_carrier']")
        self.carrier_pose = sdf_pose(carrier) @ sdf_pose(carrier.find("link"))
        self.node = rclpy.create_node("harness_arm_experiment")
        self.transport = Node()
        self.joints, self.poses = {}, None
        self.phase = "startup"
        self.contacts, self.contact_count, self.unintended = {}, 0, []
        self.stop_diagnostics = None
        self.evaluation_guards = []
        self.motion_fingers = None
        self.expected_contact_publishers = sum(
            s.findtext("topic") == CONTACT_TOPIC
            for s in ET.parse(world).getroot().findall(".//sensor[@type='contact']")
        )
        if not self.expected_contact_publishers:
            raise ValueError("Arm contact sensors required")
        self.contact_publishers, self.contact_check_time = 0, -float("inf")
        self.history, self.preflights = [], []
        self.targets = None
        self.grips = {n: OPEN_TRAVEL_M for n in self.arms}
        self.pubs = {}
        command = ["ros2", "run", "ros_gz_bridge", "parameter_bridge"]
        for name in self.arms:
            self.node.create_subscription(
                JointState,
                f"/{name}/joints",
                lambda msg, n=name: self.receive_joints(n, msg),
                qos_profile_sensor_data,
            )
            command.append(f"/{name}/joints@sensor_msgs/msg/JointState[gz.msgs.Model")
            topics = [f"/{name}/joint_{i}/cmd" for i in range(6)] + [
                f"/{name}/gripper/{f}/cmd" for f in ("left", "right")
            ]
            self.pubs[name] = [self.node.create_publisher(Float64, topic, 10) for topic in topics]
            command += [topic + "@std_msgs/msg/Float64]gz.msgs.Double" for topic in topics]
        self.pose_topic = "/world/multi_connector_factory/dynamic_pose/info"
        self.transport.subscribe(Pose_V, self.pose_topic, self.receive_poses)
        self.transport.subscribe(Contacts, CONTACT_TOPIC, self.receive_contacts)
        self.bridge = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)

    def receive_joints(self, name, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(key in positions for key in self.arms[name].names):
            self.joints[name] = (time.monotonic(), positions)

    def receive_poses(self, msg):
        self.poses = (
            time.monotonic(),
            msg.header.stamp.sec + msg.header.stamp.nsec / 1e9,
            {
                p.name: {
                    "xyz": [p.position.x, p.position.y, p.position.z],
                    "q": [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w],
                }
                for p in msg.pose
            },
        )

    def receive_contacts(self, msg):
        self.contact_count += 1
        for c in msg.contact:
            first, second = c.collision1.name, c.collision2.name
            accumulate_contact(
                self.contacts,
                self.phase,
                first,
                second,
                c.depth,
                msg.header.stamp.sec + msg.header.stamp.nsec / 1e9,
            )
            fixture = any(
                token in name.split("::")
                for name in (first, second)
                for token in ("fixtures", "conveyor_carrier")
            )
            if inter_arm_pair(first, second) or fixture:
                self.unintended.append([first, second])
                del self.unintended[20:]

    def snapshot(self):
        now = time.monotonic()
        if (
            not self.poses
            or set(self.joints) != set(self.arms)
            or min([self.poses[0], *[s[0] for s in self.joints.values()]]) < now - 0.5
        ):
            return None
        states = {}
        poses = self.poses[2]
        if "conveyor_carrier" not in poses or "carrier" not in poses:
            return None
        model, link = poses["conveyor_carrier"], poses["carrier"]
        base_r = quaternion_rotation(model["q"])
        carrier_position = np.array(model["xyz"]) + base_r @ link["xyz"]
        carrier_rotation = base_r @ quaternion_rotation(link["q"])
        carrier_error = float(np.linalg.norm(carrier_position - self.carrier_pose[:3, 3]))
        carrier_angle = float(
            np.linalg.norm(rotation_error(self.carrier_pose[:3, :3], carrier_rotation))
        )
        if carrier_error > 0.0005 or carrier_angle > 0.002:
            self.stop_diagnostics = {
                "phase": self.phase,
                "sim_time_s": self.poses[1],
                "carrier_position_error_m": carrier_error,
                "carrier_angle_error_rad": carrier_angle,
                "carrier_xyz_m": carrier_position.tolist(),
            }
            raise RuntimeError("Carrier moved outside the preflight geometry tolerance")
        for name, arm in self.arms.items():
            q = np.array([self.joints[name][1][key] for key in arm.names])
            fingers = [name + "_finger_" + f + "_joint" for f in ("left", "right")]
            flange = name + "_rv4frl_hand_flange"
            if (
                name not in poses
                or flange not in poses
                or not all(f in self.joints[name][1] for f in fingers)
            ):
                return None
            parent, link = poses[name], poses[flange]
            base_r = quaternion_rotation(parent["q"])
            tool_r = base_r @ quaternion_rotation(link["q"])
            position = np.array(parent["xyz"]) + base_r @ link["xyz"] + tool_r @ arm.tool[:3, 3]
            fk = arm.forward(q)
            fk_error = float(np.linalg.norm(fk[:3, 3] - position))
            states[name] = {
                "q_rad": q.tolist(),
                "fingers": [self.joints[name][1][f] for f in fingers],
                "tcp": position.tolist(),
                "tcp_rotation_world": tool_r.tolist(),
                "fk_native_error_m": fk_error,
                "command_q_rad": self.targets[name].tolist() if self.targets is not None else None,
                "command_tcp_pose": self.arms[name].forward(self.targets[name]).tolist()
                if self.targets is not None
                else None,
            }
        return {
            "time_s": now,
            "sim_time_s": self.poses[1],
            "arms": states,
            "evaluation_poses": poses,
            "phase": self.phase,
        }

    def contact_monitor_ready(self):
        now = time.monotonic()
        if now - self.contact_check_time >= 1:
            self.contact_publishers = len(self.transport.topic_info(CONTACT_TOPIC)[0])
            self.contact_check_time = now
        return self.contact_publishers >= self.expected_contact_publishers

    def wait_ready(self):
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            row = self.snapshot()
            if (
                row
                and self.contact_monitor_ready()
                and all(p.get_subscription_count() for ps in self.pubs.values() for p in ps)
            ):
                self.targets = {n: np.array(s["q_rad"]) for n, s in row["arms"].items()}
                return row
        raise RuntimeError("Missing fresh joint, native robot pose, contact or command feedback")

    def check_finger_intervals(self, row):
        if self.motion_fingers is not None and row:
            for name, intervals in self.motion_fingers.items():
                values = np.array(row["arms"][name]["fingers"])
                if np.any(values < intervals[:, 0]) or np.any(values > intervals[:, 1]):
                    self.stop_diagnostics = {
                        "phase": self.phase,
                        "sim_time_s": row["sim_time_s"],
                        "arm": name,
                        "finger_positions_m": values.tolist(),
                        "planned_finger_intervals_m": intervals.tolist(),
                    }
                    raise RuntimeError("Finger left its measured collision interval")

    def tick(self, *, teardown=False):
        if not teardown and not self.contact_monitor_ready():
            raise RuntimeError("Lost arm contact publishers")
        if self.unintended and not teardown:
            raise RuntimeError("Unintended arm/fixture contact")
        if self.snapshot() is None and not teardown:
            # Drain queued feedback before declaring loss; never move on stale state.
            deadline = time.monotonic() + 2
            while self.snapshot() is None and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.01)
            if self.snapshot() is None:
                raise RuntimeError("Lost feedback")
        if not teardown:
            self.check_finger_intervals(self.snapshot())
        for n, pubs in self.pubs.items():
            for pub, value in zip(pubs, [*self.targets[n], self.grips[n], self.grips[n]]):
                pub.publish(Float64(data=float(value)))
        rclpy.spin_once(self.node, timeout_sec=0.02)
        row = self.snapshot()
        if row and (
            not self.history
            or row["sim_time_s"] - self.history[-1]["sim_time_s"] >= 0.1
            or row["phase"] != self.history[-1]["phase"]
        ):
            self.history.append(row)
        if row and not teardown:
            self.check_finger_intervals(row)
            for guard in self.evaluation_guards:
                guard(row)
        return row

    def dwell(self, phase, duration, *, teardown=False):
        self.phase = phase
        start, wall = self.snapshot()["sim_time_s"], time.monotonic()
        rows, previous = [], None
        while self.poses[1] - start < duration:
            if time.monotonic() - wall > 30 + 15 * duration:
                raise RuntimeError("Simulation stalled in " + phase)
            row = self.tick(teardown=teardown)
            if row and row["sim_time_s"] != previous:
                rows.append(row)
                previous = row["sim_time_s"]
        if len(rows) < 10:
            raise RuntimeError("Insufficient fresh observations")
        return rows

    def move(
        self, phase, paths, *, cartesian_speed_m_s=None, settle_time_s=20, full_finger_stroke=False
    ):
        if not np.isfinite(settle_time_s) or not 0 < settle_time_s <= 60:
            raise ValueError("Settling time must be finite and within (0, 60] seconds")
        if cartesian_speed_m_s is not None and (
            not np.isfinite(cartesian_speed_m_s) or cartesian_speed_m_s <= 0
        ):
            raise ValueError("Positive finite Cartesian speed required")
        self.phase = phase
        self.tick()
        row = self.snapshot()
        parked = {n: np.array(s["q_rad"]) for n, s in row["arms"].items() if n not in paths}
        paths = {n: np.asarray(p, dtype=float) for n, p in paths.items()}
        for n, path in paths.items():
            if np.max(np.abs(path[0] - row["arms"][n]["q_rad"])) > 0.02:
                raise RuntimeError("Prepared path start does not match feedback")
        full = with_parked_arms(paths, parked)
        self.motion_fingers = {
            n: finger_collision_intervals(s["fingers"], full_stroke=full_finger_stroke)
            for n, s in row["arms"].items()
        }
        state_bounds = {
            n: FingerStateBounds(
                b,
                {
                    n + "_finger_" + f + "_joint": self.motion_fingers[n][i]
                    for i, f in enumerate(("left", "right"))
                },
            )
            for n, b in self.bounds.items()
        }
        checks = {
            "phase": phase,
            "settle_time_s": float(settle_time_s),
            "arms": screen_paths(state_bounds, full),
            "fixtures": self.fixtures.screen(state_bounds, full, margin=0.001),
            "finger_envelope_mode": "full_mechanical_stroke"
            if full_finger_stroke
            else "measured_plus_minus_0.5mm",
            "planned_finger_intervals_m": {n: v.tolist() for n, v in self.motion_fingers.items()},
        }
        self.preflights.append(checks)
        if not all(checks[k]["clear"] for k in ("arms", "fixtures")):
            raise RuntimeError("Motion preflight rejected: " + str(checks))
        duration = max(
            [
                3.0,
                *[
                    sum(np.max(np.abs(b - a)) for a, b in zip(p[:-1], p[1:])) / 0.35
                    for p in paths.values()
                ],
            ]
        )
        if cartesian_speed_m_s is not None:
            for name, path in paths.items():
                positions = [self.arms[name].forward(q)[:3, 3] for q in path]
                duration = max(
                    duration,
                    sum(np.linalg.norm(b - a) for a, b in zip(positions[:-1], positions[1:]))
                    / cartesian_speed_m_s,
                )
        start, wall, stable, previous = row["sim_time_s"], time.monotonic(), 0, None
        goals = {n: self.arms[n].forward(p[-1]) for n, p in paths.items()}
        while self.poses[1] - start < duration + settle_time_s and time.monotonic() - wall < 240:
            t = min(1, (self.poses[1] - start) / duration)
            self.targets.update(
                {n: interpolate_joints(p, t * t * (3 - 2 * t)) for n, p in paths.items()}
            )
            actual = self.tick()
            if not actual:
                continue
            if any(
                np.max(np.abs(np.array(actual["arms"][n]["q_rad"]) - q)) > 0.01
                for n, q in parked.items()
            ):
                raise RuntimeError("Parked arm moved")
            valid = t >= 1 and all(
                actual["arms"][n]["fk_native_error_m"] < 0.001
                and np.linalg.norm(np.array(actual["arms"][n]["tcp"]) - g[:3, 3]) < 0.002
                and np.linalg.norm(
                    rotation_error(g[:3, :3], np.array(actual["arms"][n]["tcp_rotation_world"]))
                )
                < 0.025
                and np.max(np.abs(np.array(actual["arms"][n]["q_rad"]) - paths[n][-1])) < 0.01
                for n, g in goals.items()
            )
            if actual["sim_time_s"] != previous:
                stable = stable + 1 if valid else 0
                previous = actual["sim_time_s"]
            if stable >= 5:
                self.motion_fingers = None
                return actual
        self.stop_diagnostics = {
            "phase": phase,
            "sim_time_s": self.poses[1],
            "command_targets_rad": {n: q.tolist() for n, q in self.targets.items()},
            "goal_tcp_poses": {n: g.tolist() for n, g in goals.items()},
            "last_observation": actual,
        }
        raise RuntimeError("Robot tracking did not converge in " + phase)

    def close(self):
        try:
            if self.targets is not None:
                # Release fingers even when geometry/freshness checks caused
                # the failure. Never let a snapshot exception suppress teardown.
                self.phase = "final_open"
                start = self.poses[1] if self.poses else None
                deadline = time.monotonic() + 30
                while time.monotonic() < deadline:
                    for pubs in self.pubs.values():
                        for pub in pubs[-2:]:
                            pub.publish(Float64(data=OPEN_TRAVEL_M))
                    rclpy.spin_once(self.node, timeout_sec=0.02)
                    if self.poses:
                        if start is None:
                            start = self.poses[1]
                        if self.poses[1] - start >= 1:
                            break
                else:
                    raise RuntimeError("Final open sent but simulation did not advance")
        finally:
            self.transport.unsubscribe(self.pose_topic)
            self.transport.unsubscribe(CONTACT_TOPIC)
            stop_process(self.bridge, group=True)
            self.node.destroy_node()
