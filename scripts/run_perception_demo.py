"""Start the MELFA RGB-D scene and acquire randomized harness connector observations."""

import argparse
import math
import secrets
import shlex
import subprocess
from pathlib import Path

from local_vision import MODEL
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
    parser.add_argument("--verify-held-pose", action="store_true")
    parser.add_argument("--inspection-calibration", type=Path)
    parser.add_argument("--end-markers", action="store_true")
    parser.add_argument("--verify-before-close", action="store_true")
    parser.add_argument("--correct-before-close", action="store_true")
    parser.add_argument("--headless", action="store_true", help="Run Gazebo server without its GUI")
    parser.add_argument(
        "--release-ai-memory",
        action="store_true",
        help="Use the local Ollama CLI to unload the drawing model before robot control",
    )
    parser.add_argument("--inspection-timeout", type=float, default=90)
    parser.add_argument("--fence-width-mm", type=float, default=12)
    parser.add_argument("--front-fence-side", choices=("both", "left", "right"), default="both")
    parser.add_argument(
        "--capture-gripper",
        action="store_true",
        help="Experimental fore/aft finger fences around a 21 mm housing slot",
    )
    parser.add_argument(
        "--backstop-gripper",
        action="store_true",
        help="Experimental rear-face finger fences to resist rotation",
    )
    parser.add_argument(
        "--gripper-p-gain",
        type=float,
        default=1000,
        help="Prototype jaw position gain; force limits remain 12 N per finger",
    )
    parser.add_argument(
        "--place-in-nest",
        action="store_true",
        help="After paired pickup, attempt a continuously planned placement and primary release in the nest",
    )
    args = parser.parse_args()
    if not math.isfinite(args.inspection_timeout) or not 5 <= args.inspection_timeout <= 300:
        parser.error("--inspection-timeout must be finite and between 5 and 300 seconds")
    if not math.isfinite(args.fence_width_mm) or not 4 <= args.fence_width_mm <= 12:
        parser.error("--fence-width-mm must be finite and between 4 and 12")
    if args.fence_width_mm != 12 and not (args.capture_gripper or args.backstop_gripper):
        parser.error("--fence-width-mm requires capture or backstop gripper")
    if args.capture_gripper and args.backstop_gripper:
        parser.error("Choose one experimental finger profile")
    if args.front_fence_side != "both" and not args.capture_gripper:
        parser.error("--front-fence-side requires --capture-gripper")
    if not math.isfinite(args.gripper_p_gain) or args.gripper_p_gain <= 0:
        parser.error("--gripper-p-gain must be finite and positive")
    if args.place_in_nest and (
        not args.regrasp_nest or not args.support_plug or not args.pick_connector
    ):
        parser.error("--place-in-nest requires --regrasp-nest, --pick-connector and --support-plug")
    if args.support_plug and not args.pick_connector:
        parser.error("--support-plug requires --pick-connector")
    if args.correct_before_close and not args.verify_before_close:
        parser.error("--correct-before-close requires --verify-before-close")
    if args.verify_held_pose and not args.pick_connector:
        parser.error("--verify-held-pose requires --pick-connector")
    if args.verify_before_close and (
        not args.pick_connector or not args.inspection_calibration or not args.end_markers
    ):
        parser.error(
            "--verify-before-close requires pickup, inspection calibration and end markers"
        )
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
    if args.gripper_p_gain != 1000:
        models += "_g" + format(args.gripper_p_gain, "g").replace(".", "p")
    if args.backstop_gripper:
        models += "_backstop"
    if args.capture_gripper:
        models += "_capture"
        if args.front_fence_side != "both":
            models += "_front_" + args.front_fence_side
    if args.fence_width_mm != 12:
        models += "_fence" + format(args.fence_width_mm, "g").replace(".", "p")
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
    if args.end_markers:
        generate.append("--end-markers")
    if args.inspection_calibration is not None:
        generate.extend(["--inspection-calibration", str(args.inspection_calibration)])
    factory = ["python3", "scripts/build_factory.py", "--robot", "melfa_rv4frl", "--output", models]
    factory.extend(
        ["--gripper-p-gain", str(args.gripper_p_gain), "--fence-width-mm", str(args.fence_width_mm)]
    )
    if args.backstop_gripper:
        factory.append("--backstop-gripper")
    if args.capture_gripper:
        factory.append("--capture-gripper")
        factory.extend(["--front-fence-side", args.front_fence_side])
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
            ["gz", "sim", "-r", str(case / "world.sdf")]
            + (["-s"] if args.headless else ["--gui-config", "sim/melfa_gui.config"])
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
    if not args.headless:
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
        if args.release_ai_memory:
            run("ollama", "stop", MODEL)
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
            if args.place_in_nest:
                pickup.append("--place-in-nest")
            if args.verify_before_close:
                pickup.append("--verify-before-close")
            if args.correct_before_close:
                pickup.append("--correct-before-close")
            if args.verify_held_pose:
                pickup.append("--verify-held-pose")
            pickup.extend(["--inspection-timeout", str(args.inspection_timeout)])
            if args.verify_held_pose or args.verify_before_close:
                if args.end_markers:
                    pickup.append("--held-end-markers")
                if args.inspection_calibration is not None:
                    pickup.extend(["--held-calibration", str(case / "inspection_camera.json")])
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
