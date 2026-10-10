"""Run local image reading, validation, then ROS 2 / Gazebo insertion."""

import argparse
import subprocess
from pathlib import Path

from local_vision import MODEL
from project_paths import ROOT
from read_drawing import read_drawing


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("image", type=Path)
    parser.add_argument("--model", default=MODEL)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--keep-open", action="store_true")
    args = parser.parse_args()
    task_path = ROOT / "outputs/current_ai_task.json"
    read_drawing(args.image, task_path, args.model)
    runner = ["python3", "scripts/run_insertion.py", "--task", "outputs/current_ai_task.json"]
    if args.gui:
        command = [
            "docker",
            "compose",
            "--profile",
            "gui",
            "exec",
            "-T",
            "gui",
            "/ros_entrypoint.sh",
        ]
        runner += ["--gui", "--start-delay", "10"]
    else:
        command = ["docker", "compose", "run", "--rm", "dev"]
    if args.keep_open:
        runner += ["--keep-open"]
    subprocess.run(command + runner, cwd=ROOT, check=True)


if __name__ == "__main__":
    main()
