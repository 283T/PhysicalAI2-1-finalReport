"""Check Gazebo physics startup and Gazebo-to-ROS clock delivery."""

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


def main():
    env = os.environ.copy()
    env["GZ_PARTITION"] = f"pai-check-{os.getpid()}"
    clock_topic = f"/pai_check_{os.getpid()}/clock"
    world = Path(__file__).resolve().parents[1] / "sim/worlds/smoke.sdf"
    commands = [
        ["gz", "sim", "-s", "-r", str(world)],
        ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
         "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
         "--ros-args", "-r", f"/clock:={clock_topic}"],
    ]
    processes = []
    rclpy.init()
    node = rclpy.create_node(f"pai_environment_check_{os.getpid()}")
    received = []
    node.create_subscription(
        Clock, clock_topic,
        lambda msg: received.append(msg.clock.sec * 1_000_000_000 + msg.clock.nanosec),
        qos_profile_sensor_data,
    )
    with tempfile.TemporaryFile(mode="w+") as log:
        try:
            for command in commands:
                processes.append(subprocess.Popen(
                    command, env=env, stdout=log, stderr=log,
                    start_new_session=True,
                ))
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                if any(process.poll() is not None for process in processes):
                    raise RuntimeError("Gazebo or ROS bridge exited early")
                rclpy.spin_once(node, timeout_sec=0.2)
                if len(received) >= 2 and received[-1] > received[0]:
                    print("PASS: Gazebo simulation clock received through ROS 2 bridge")
                    return
            raise RuntimeError("No advancing simulation clock received within 30 seconds")
        except Exception:
            log.seek(0)
            print(log.read()[-8000:])
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
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
