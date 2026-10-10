"""ROS 2 controlled pick-and-insert using a validated assembly instruction."""

import argparse
import json
import math
import os
import secrets
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseArray
from processes import stop_process
from project_paths import ROOT
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64, String
from task_schema import validate_task


class Insertion:
    def __init__(self):
        self.node = rclpy.create_node("harness_connection")
        self.joints = {}
        self.pose = None
        self.grasp_state = None
        self.node.create_subscription(
            JointState, "/gantry/joints", self.on_joints, qos_profile_sensor_data
        )
        self.node.create_subscription(
            PoseArray, "/harness/pose", self.on_pose, qos_profile_sensor_data
        )
        self.node.create_subscription(String, "/grasp/state", self.on_grasp, 10)
        self.commands = {
            axis: self.node.create_publisher(Float64, f"/gantry/{axis}/cmd", 10) for axis in "xyz"
        }
        self.attach = self.node.create_publisher(Empty, "/grasp/attach", 10)
        self.detach = self.node.create_publisher(Empty, "/grasp/detach", 10)

    def on_joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def on_pose(self, msg):
        if msg.poses:
            self.pose = msg.poses[0]

    def on_grasp(self, msg):
        self.grasp_state = msg.data

    def wait(self, predicate, description, timeout=45):
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
        self.wait(
            lambda: all(
                abs(self.joints.get(f"{axis}_joint", math.inf) - value) < 0.0005
                for axis, value in goals.items()
            ),
            label,
        )
        self.settle(0.3)

    def run(self, task):
        self.wait(
            lambda: (
                self.pose is not None
                and len(self.joints) >= 3
                and all(pub.get_subscription_count() for pub in self.commands.values())
            ),
            "initial feedback",
        )
        self.settle()
        self.grasp(False)  # Gazebo detachable joints are initially attached.
        self.settle()
        p = self.pose.position
        start = [p.x, p.y, p.z]
        self.move("1. Approach supply", start[0], start[1], start[2] + 0.03)
        self.grasp(True)
        self.move("2. Lift", start[0], start[1], 0.30)
        if self.pose.position.z < start[2] + 0.05:
            raise RuntimeError("Harness plug did not lift with the tool")
        origin = task["board_origin_m"]
        x, y = origin[0] + task["x_mm"] / 1000, origin[1] + task["y_mm"] / 1000
        self.move("3. Transfer above connection panel", x, y, 0.30)
        self.move("4. Approach receptacle", x, y, 0.20)
        self.move("5. Connect harness plug", x, y, origin[2] + 0.04)
        self.grasp(False)
        self.move("6. Retract", x, y, 0.30)
        self.settle(1)
        p = self.pose.position
        q = self.pose.orientation
        depth = origin[2] - (p.z - 0.05)
        result = {
            "task_source": task["source"],
            "drawing_id": task.get("drawing_id"),
            "socket": task.get("socket"),
            "harness_id": task["harness_id"],
            "plug_id": task["plug_id"],
            "connection_check": "mechanical seating only",
            "position_m": [p.x, p.y, p.z],
            "target_m": [x, y, origin[2] + 0.01],
            "pin_insertion_depth_m": depth,
            "grasp_state": self.grasp_state,
            "simplifications": [
                "fixed-joint grasp",
                "enlarged pins and holes",
                "fixed supply and panel",
                "rigid visual cable; no cable forces",
            ],
        }
        if task["source"] == "local_vision_model":
            result["model"] = task.get("model")
            result["image_sha256"] = task.get("image_sha256")
        result["success"] = (
            abs(p.x - x) < 0.001
            and abs(p.y - y) < 0.001
            and abs(p.z - (origin[2] + 0.01)) < 0.001
            and abs(q.x) < 0.02
            and abs(q.y) < 0.02
            and abs(q.z) < 0.02
            and depth > 0.03
            and self.grasp_state == "detached"
        )
        output = ROOT / "outputs"
        output.mkdir(exist_ok=True)
        text = json.dumps(result, indent=2) + "\n"
        (output / "insertion_result.json").write_text(text)
        print(json.dumps(result, indent=2), flush=True)
        if not result["success"]:
            raise RuntimeError("Released harness plug is not seated in the receptacle")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", type=Path, default=ROOT / "examples/task_a.json")
    parser.add_argument("--world", type=Path, default=ROOT / "sim/worlds/insertion.sdf")
    parser.add_argument("--gui", action="store_true", help="Show Gazebo on the X display")
    parser.add_argument(
        "--start-delay", type=float, default=0, help="Wait this many seconds before robot motion"
    )
    parser.add_argument(
        "--keep-open", action="store_true", help="Keep the finished simulation open until Ctrl+C"
    )
    args = parser.parse_args()
    task = json.loads(args.task.read_text())
    board = json.loads((ROOT / "configs/board.json").read_text())
    socket = validate_task(task, board)
    task["socket"] = socket["id"]
    if not math.isfinite(args.start_delay) or args.start_delay < 0:
        raise ValueError("Start delay must be finite and nonnegative")
    os.environ.setdefault("ROS_DOMAIN_ID", str(secrets.randbelow(200) + 1))
    env = os.environ.copy()
    env["GZ_PARTITION"] = f"pai-insertion-{os.getpid()}"
    bridges = [f"/gantry/{axis}/cmd@std_msgs/msg/Float64]gz.msgs.Double" for axis in "xyz"]
    bridges += [
        "/gantry/joints@sensor_msgs/msg/JointState[gz.msgs.Model",
        "/harness/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
        "/grasp/attach@std_msgs/msg/Empty]gz.msgs.Empty",
        "/grasp/detach@std_msgs/msg/Empty]gz.msgs.Empty",
        "/grasp/state@std_msgs/msg/String[gz.msgs.StringMsg",
    ]
    gazebo = ["gz", "sim", "-r", str(args.world)]
    if args.gui:
        gazebo += ["--render-engine", "ogre", "--gui-config", str(ROOT / "sim/gui.config")]
    else:
        gazebo += ["-s"]
    commands = [gazebo, ["ros2", "run", "ros_gz_bridge", "parameter_bridge", *bridges]]
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
                processes.append(
                    subprocess.Popen(
                        command, env=env, stdout=log, stderr=log, start_new_session=True
                    )
                )
            if args.start_delay > 0:
                print(f"Starting in {args.start_delay:g} seconds", flush=True)
                demo.settle(args.start_delay)
            demo.run(task)
            (output / f"result_{socket['id']}.json").write_text(result_path.read_text())
            if args.keep_open:
                print("Demo complete. Ctrl+C to close the simulation.", flush=True)
                while True:
                    rclpy.spin_once(demo.node, timeout_sec=0.2)
        except KeyboardInterrupt:
            pass
        except Exception as error:
            result_path.write_text(
                json.dumps({"success": False, "error": str(error)}, indent=2) + "\n"
            )
            raise
        finally:
            for process in reversed(processes):
                stop_process(process, group=True)
            demo.node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
