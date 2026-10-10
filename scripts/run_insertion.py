"""ROS 2 controlled pick-and-insert baseline; AI is not connected yet."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64, String


ROOT = Path(__file__).resolve().parents[1]


class Insertion:
    def __init__(self):
        self.node = rclpy.create_node("pcb_insertion_baseline")
        self.joints = {}
        self.pose = None
        self.grasp_state = None
        self.node.create_subscription(JointState, "/gantry/joints", self.on_joints,
                                      qos_profile_sensor_data)
        self.node.create_subscription(PoseArray, "/connector/pose", self.on_pose,
                                      qos_profile_sensor_data)
        self.node.create_subscription(String, "/grasp/state", self.on_grasp, 10)
        self.commands = {axis: self.node.create_publisher(Float64, f"/gantry/{axis}/cmd", 10)
                         for axis in "xyz"}
        self.attach = self.node.create_publisher(Empty, "/grasp/attach", 10)
        self.detach = self.node.create_publisher(Empty, "/grasp/detach", 10)

    def on_joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def on_pose(self, msg):
        if msg.poses:
            self.pose = msg.poses[0]

    def on_grasp(self, msg):
        self.grasp_state = msg.data

    def wait(self, predicate, description, timeout=20):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return
        raise RuntimeError(f"Timeout: {description}; joints={self.joints}; pose={self.pose}")

    def settle(self, seconds=0.5):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def grasp(self, attached):
        pub = self.attach if attached else self.detach
        self.wait(lambda: pub.get_subscription_count() > 0, "grasp bridge connected")
        pub.publish(Empty())
        state = "attached" if attached else "detached"
        self.wait(lambda: self.grasp_state == state, state)

    def move(self, label, x, y, z):
        print(label, flush=True)
        goals = dict(zip("xyz", (x + 0.25, y, z - 0.2)))
        for axis, value in goals.items():
            self.commands[axis].publish(Float64(data=value))
        self.wait(lambda: all(abs(self.joints.get(f"{axis}_joint", math.inf) - value) < 0.0005
                              for axis, value in goals.items()), label)
        self.settle(0.3)

    def run(self, task):
        self.wait(lambda: self.pose is not None and len(self.joints) >= 3
                  and all(pub.get_subscription_count() for pub in self.commands.values()),
                  "initial feedback")
        self.settle()
        self.grasp(False)  # Gazebo detachable joints are initially attached.
        self.settle()
        p = self.pose.position
        start = [p.x, p.y, p.z]
        self.move("1. Approach supply", start[0], start[1], start[2] + 0.03)
        self.grasp(True)
        self.move("2. Lift", start[0], start[1], 0.30)
        if self.pose.position.z < start[2] + 0.05:
            raise RuntimeError("Connector did not lift with the tool")
        origin = task["board_origin_m"]
        x, y = origin[0] + task["x_mm"] / 1000, origin[1] + task["y_mm"] / 1000
        self.move("3. Transfer above PCB", x, y, 0.30)
        self.move("4. Approach holes", x, y, 0.20)
        self.move("5. Insert pins", x, y, origin[2] + 0.04)
        self.grasp(False)
        self.move("6. Retract", x, y, 0.30)
        self.settle(1)
        p = self.pose.position
        q = self.pose.orientation
        depth = origin[2] - (p.z - 0.05)
        result = {"task_source": task["source"], "position_m": [p.x, p.y, p.z],
                  "target_m": [x, y, origin[2] + 0.01], "pin_insertion_depth_m": depth,
                  "grasp_state": self.grasp_state,
                  "simplifications": ["fixed-joint grasp", "enlarged pins and holes",
                                      "fixed supply and board", "no AI yet"]}
        result["success"] = (abs(p.x-x) < 0.001 and abs(p.y-y) < 0.001
                             and abs(p.z-(origin[2]+0.01)) < 0.001
                             and abs(q.x) < 0.02 and abs(q.y) < 0.02
                             and abs(q.z) < 0.02 and depth > 0.03
                             and self.grasp_state == "detached")
        output = ROOT / "outputs"
        output.mkdir(exist_ok=True)
        (output / "insertion_result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2), flush=True)
        if not result["success"]:
            raise RuntimeError("Released connector is not seated in the PCB holes")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", type=Path, default=ROOT / "examples/task_a.json")
    parser.add_argument("--world", type=Path, default=ROOT / "sim/worlds/insertion.sdf")
    args = parser.parse_args()
    task = json.loads(args.task.read_text())
    if task["component"] != "connector" or task["yaw_deg"] != 0:
        raise ValueError("Baseline supports one connector with yaw=0 only")
    if any(not math.isfinite(v) for v in (*task["board_origin_m"], task["x_mm"], task["y_mm"])):
        raise ValueError("Task coordinates must be finite")
    # This scene currently has a single known socket. Reject any other target.
    if task["board_origin_m"] != [0.19, 0.10, 0.12] or (task["x_mm"], task["y_mm"]) != (60, 50):
        raise ValueError("Target does not match the baseline PCB socket")
    env = os.environ.copy()
    env["GZ_PARTITION"] = f"pai-insertion-{os.getpid()}"
    bridges = [f"/gantry/{axis}/cmd@std_msgs/msg/Float64]gz.msgs.Double" for axis in "xyz"]
    bridges += ["/gantry/joints@sensor_msgs/msg/JointState[gz.msgs.Model",
                "/connector/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
                "/grasp/attach@std_msgs/msg/Empty]gz.msgs.Empty",
                "/grasp/detach@std_msgs/msg/Empty]gz.msgs.Empty",
                "/grasp/state@std_msgs/msg/String[gz.msgs.StringMsg"]
    commands = [["gz", "sim", "-s", "-r", str(args.world)],
                ["ros2", "run", "ros_gz_bridge", "parameter_bridge", *bridges]]
    processes = []
    rclpy.init()
    demo = Insertion()
    output = ROOT / "outputs"
    output.mkdir(exist_ok=True)
    result_path = output / "insertion_result.json"
    result_path.unlink(missing_ok=True)
    with (output / "simulation.log").open("w") as log:
        try:
            for command in commands:
                processes.append(subprocess.Popen(command, env=env, stdout=log, stderr=log,
                                                  start_new_session=True))
            demo.run(task)
        except Exception as error:
            result_path.write_text(json.dumps({"success": False, "error": str(error)}, indent=2) + "\n")
            raise
        finally:
            for process in reversed(processes):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
            demo.node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
