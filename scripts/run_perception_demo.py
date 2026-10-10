"""Start the MELFA RGB-D scene and acquire randomized harness connector observations."""

import argparse
import secrets
import shlex
import subprocess
from pathlib import Path

from project_paths import ROOT


def run(*command):
    subprocess.run(command, cwd=ROOT, check=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--design-seed", type=int, default=secrets.randbelow(2**31))
    parser.add_argument("--placement-seed", type=int, default=secrets.randbelow(2**31))
    parser.add_argument(
        "--evaluate",
        action="store_true",
        help="Use independent ground truth to measure accuracy only",
    )
    parser.add_argument(
        "--plan-reach", action="store_true", help="Read the drawing locally and calculate hover IK"
    )
    parser.add_argument(
        "--check-reach",
        action="store_true",
        help="Also replay IK in isolated single-arm validation scenes",
    )
    parser.add_argument("--output", type=Path, default=Path("outputs/perception_demo"))
    args = parser.parse_args()
    output = (ROOT / args.output).resolve()
    if not output.is_relative_to(ROOT):
        raise ValueError("Output must be inside the mounted project workspace")
    relative = output.relative_to(ROOT)
    case = relative / "case"
    observations = relative / "observations"
    if not (ROOT / "outputs/melfa_install/setup.bash").exists():
        run("bash", "scripts/setup_melfa.sh")
    generate = [
        "python3",
        "scripts/build_case.py",
        "--factory-world",
        "outputs/melfa_worlds/factory_prototype.sdf",
        "--design-seed",
        str(args.design_seed),
        "--placement-seed",
        str(args.placement_seed),
        "--output",
        str(case),
    ]
    run(
        "docker",
        "compose",
        "run",
        "--rm",
        "dev",
        "bash",
        "-c",
        "source outputs/melfa_install/setup.bash && "
        "python3 scripts/build_factory.py --robot melfa_rv4frl --output outputs/melfa_worlds && "
        + shlex.join(generate),
    )
    run("docker", "compose", "--profile", "gui", "up", "-d", "gui")
    # Recreate this project's desktop session to stop the previous Gazebo scene.
    run("docker", "compose", "restart", "gui")
    run(
        "docker",
        "compose",
        "exec",
        "-T",
        "gui",
        "python3",
        "scripts/start_desktop.py",
        "--wait-only",
    )
    start = (
        "source /opt/ros/jazzy/setup.bash && source outputs/melfa_install/setup.bash && "
        + shlex.join(
            ["gz", "sim", "-r", str(case / "world.sdf"), "--gui-config", "sim/melfa_gui.config"]
        )
        + " > "
        + shlex.quote(str(case / "gazebo.log"))
        + " 2>&1"
    )
    run("docker", "compose", "exec", "-d", "gui", "bash", "-c", start)
    command = [
        "python3",
        "scripts/evaluate_perception.py" if args.evaluate else "scripts/observe_connectors.py",
        "--output",
        str(observations),
    ]
    if not args.evaluate:
        command.append("--bridge")
    run(
        "docker",
        "compose",
        "exec",
        "-T",
        "gui",
        "bash",
        "-c",
        "source /opt/ros/jazzy/setup.bash && " + shlex.join(command),
    )
    if args.plan_reach or args.check_reach:
        plan = relative / "connection_plan.json"
        reach = relative / "reach_plan.json"
        run(
            "python3",
            "scripts/read_connection_plan.py",
            str(case / "drawing.png"),
            "--board",
            str(case / "board.json"),
            "--output",
            str(plan),
        )
        run(
            "docker",
            "compose",
            "run",
            "--rm",
            "dev",
            "python3",
            "scripts/plan_reach.py",
            "--plan",
            str(plan),
            "--board",
            str(case / "board.json"),
            "--detections",
            str(observations / "detections.json"),
            "--world",
            str(case / "world.sdf"),
            "--models",
            "outputs/melfa_worlds",
            "--output",
            str(reach),
        )
        if args.check_reach:
            run(
                "docker",
                "compose",
                "run",
                "--rm",
                "dev",
                "python3",
                "scripts/check_reach.py",
                "--plan",
                str(reach),
                "--world",
                str(case / "world.sdf"),
                "--models",
                "outputs/melfa_worlds",
                "--output",
                str(relative / "reach_control"),
            )
        print(f"Reach plan: {ROOT / reach}")
    print(f"Observations: {output / 'observations/detections.json'}")


if __name__ == "__main__":
    main()
