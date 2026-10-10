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
    parser.add_argument(
        "--require-6d", action="store_true", help="Require coded-face 6D observations"
    )
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
    parser.add_argument(
        "--pick-connector",
        choices=("P1", "P2", "P3"),
        help="Attempt one fresh RGB-D-derived pickup in the running MELFA scene",
    )
    parser.add_argument(
        "--support-plug",
        choices=("P1", "P2", "P3"),
        help="Hold another terminal with the second arm during pickup",
    )
    parser.add_argument(
        "--keep-camera",
        action="store_true",
        help="Keep full RGB-D rendering rate during --pick-connector",
    )
    parser.add_argument(
        "--regrasp-nest",
        action="store_true",
        help="Add a passive nest and split finger pads; no regrasp motion is enabled",
    )
    parser.add_argument("--output", type=Path, default=Path("outputs/perception_demo"))
    args = parser.parse_args()
    if args.support_plug and not args.pick_connector:
        parser.error("--support-plug requires --pick-connector")
    if args.support_plug == args.pick_connector and args.support_plug:
        parser.error("Choose different pickup and support terminals")
    if args.pick_connector:
        args.require_6d = True
    output = (ROOT / args.output).resolve()
    if not output.is_relative_to(ROOT):
        raise ValueError("Output must be inside the mounted project workspace")
    relative = output.relative_to(ROOT)
    case = relative / "case"
    observations = relative / "observations"
    models = "outputs/melfa_regrasp_worlds" if args.regrasp_nest else "outputs/melfa_worlds"
    if not (ROOT / "outputs/melfa_install/setup.bash").exists():
        run("bash", "scripts/setup_melfa.sh")
    generate = [
        "python3",
        "scripts/build_case.py",
        "--factory-world",
        models + "/factory_prototype.sdf",
        "--design-seed",
        str(args.design_seed),
        "--placement-seed",
        str(args.placement_seed),
        "--output",
        str(case),
    ]
    if args.regrasp_nest:
        generate.append("--regrasp-nest")
    factory = ["python3", "scripts/build_factory.py", "--robot", "melfa_rv4frl", "--output", models]
    if args.regrasp_nest:
        factory.append("--split-finger-contacts")
    run(
        "docker",
        "compose",
        "run",
        "--rm",
        "dev",
        "bash",
        "-c",
        "source outputs/melfa_install/setup.bash && "
        + shlex.join(factory)
        + " && "
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
    if args.require_6d:
        command.append("--require-6d")
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
    run(
        "docker",
        "compose",
        "exec",
        "-T",
        "gui",
        "bash",
        "-c",
        "source /opt/ros/jazzy/setup.bash && python3 scripts/set_gui_view.py",
    )
    if args.plan_reach or args.check_reach or args.pick_connector:
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
            models,
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
                models,
                "--output",
                str(relative / "reach_control"),
            )
        print(f"Reach plan: {ROOT / reach}")
        if args.pick_connector:
            pickup = [
                "python3",
                "scripts/pick_connector.py",
                "--world",
                str(case / "world.sdf"),
                "--board",
                str(case / "board.json"),
                "--plan",
                str(plan),
                "--models",
                models,
                "--plug",
                args.pick_connector,
                "--output",
                str(relative / "pickup"),
            ]
            if args.support_plug:
                pickup.extend(["--support-plug", args.support_plug])
            if args.keep_camera:
                pickup.append("--keep-camera")
            run(
                "docker",
                "compose",
                "exec",
                "-T",
                "gui",
                "bash",
                "-c",
                "source /opt/ros/jazzy/setup.bash && " + shlex.join(pickup),
            )
    print(f"Observations: {output / 'observations/detections.json'}")


if __name__ == "__main__":
    main()
