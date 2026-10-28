"""Record a fresh physical dual-arm pickup/hold only; never claim connector insertion."""

import argparse
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from evidence_io import write_json_atomic
from processes import stop_process


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("world", "models", "drawing-plan", "inspection-calibration", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--wall-timeout", type=float, default=5400)
    parser.add_argument("--observation-timeout-wall-s", type=float, default=240)
    parser.add_argument("--gui-config", type=Path, default=Path("sim/melfa_gui.config"))
    args = parser.parse_args()
    if not math.isfinite(args.wall_timeout) or args.wall_timeout <= 0:
        parser.error("Positive finite recorder wall timeout required")
    if not math.isfinite(args.observation_timeout_wall_s) or not 5 <= args.observation_timeout_wall_s <= 300:
        parser.error("Observation wall timeout must be finite and within 5 to 300 seconds")
    if args.output.exists():
        parser.error("Use a new output directory and preserve earlier recordings")
    args.output.mkdir(parents=True)
    env = os.environ.copy()
    env.update(GZ_PARTITION=f"record-dual-hold-{os.getpid()}", ROS_DOMAIN_ID="174",
               FASTRTPS_DEFAULT_PROFILES_FILE="/workspace/configs/fastdds_rgbd_hd64.xml")
    sources = [*sorted(Path("scripts").glob("*.py")), args.world,
               *sorted(args.models.glob("*.urdf")), args.drawing_plan,
               args.drawing_plan.with_suffix(".audit.json"),
               args.world.parent / "board.json", args.world.parent / "drawing.png",
               args.inspection_calibration, args.gui_config]
    summary = {
        "scope": "Actual new Gazebo/ROS execution recorded at wall-clock 1x. Physical contact pickup and dual hold only; no nest transfer, board insertion or final acceptance claim.",
        "trial_state": "starting", "capture_fps": 5, "capture_speed": 1,
        "observation_timeout_wall_s": args.observation_timeout_wall_s,
        "source_sha256": {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        "environment": {k: env[k] for k in ("GZ_PARTITION", "ROS_DOMAIN_ID")},
        "raw_video": "raw.mkv", "success": False,
    }
    write_json_atomic(args.output / "recording.json", summary)
    scene = capture = controller = None
    capture_start = None
    try:
        with (args.output / "scene.log").open("w") as scene_log, \
             (args.output / "capture.log").open("w") as capture_log, \
             (args.output / "controller.log").open("w") as control_log, \
             (args.output / "phase_wall_timeline.jsonl").open("w") as timeline:
            scene = subprocess.Popen(
                ["gz", "sim", "-r", str(args.world), "--gui-config", str(args.gui_config)],
                env=env, stdout=scene_log, stderr=scene_log, start_new_session=True)
            deadline = time.monotonic() + 60
            while True:
                windows = subprocess.run(["xwininfo", "-root", "-tree"], env=env,
                                         capture_output=True, text=True)
                if "Gazebo Sim" in windows.stdout:
                    break
                if scene.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("Gazebo GUI did not become ready")
                time.sleep(0.5)
            capture_start = time.monotonic()
            capture = subprocess.Popen(
                ["ffmpeg", "-y", "-loglevel", "warning", "-f", "x11grab", "-video_size",
                 "1280x900", "-framerate", "5", "-i", ":99", "-an", "-c:v", "libx264",
                 "-preset", "ultrafast", "-crf", "24", "-pix_fmt", "yuv420p",
                 str(args.output / "raw.mkv")], env=env, stdout=capture_log,
                stderr=capture_log, start_new_session=True)
            controller = subprocess.Popen(
                [sys.executable, "scripts/dual_pickup.py", "--world", str(args.world),
                 "--models", str(args.models), "--drawing-plan", str(args.drawing_plan),
                 "--inspection-calibration", str(args.inspection_calibration),
                 "--output", str(args.output / "pickup"), "--wire-distance-mm", "140",
                 "--wire-tip-depth-mm", "19", "--centred-grips",
                 "--observation-timeout-wall-s", str(args.observation_timeout_wall_s)],
                env=env, stdout=control_log, stderr=control_log, start_new_session=True)
            summary.update(trial_state="running", scene_pid=scene.pid,
                           capture_pid=capture.pid, controller_pid=controller.pid)
            write_json_atomic(args.output / "recording.json", summary)
            print("Fresh dual-hold physical run and wall-clock recording started", flush=True)
            previous = None
            while controller.poll() is None:
                if capture.poll() is not None or scene.poll() is not None:
                    raise RuntimeError("Capture or Gazebo exited before controller completion")
                if time.monotonic() - capture_start > args.wall_timeout:
                    raise RuntimeError("Recording wall-time budget exhausted")
                progress = args.output / "pickup/progress.json"
                if progress.exists():
                    phase = json.loads(progress.read_text()).get("phase")
                    if phase != previous:
                        row = {"phase": phase, "capture_wall_offset_s": time.monotonic() - capture_start}
                        timeline.write(json.dumps(row) + "\n")
                        timeline.flush()
                        print(json.dumps(row), flush=True)
                        previous = phase
                time.sleep(1)
            summary["controller_exit_code"] = controller.returncode
            report = json.loads((args.output / "pickup/validation.json").read_text())
            summary["success"] = controller.returncode == 0 and report.get("success") is True
            summary["physical_report_success"] = report.get("success")
    except (Exception, KeyboardInterrupt) as error:
        summary.update(success=False, error=str(error) or type(error).__name__)
    finally:
        if controller is not None and controller.poll() is None:
            write_json_atomic(args.output / "external_stop.json", {
                "reason": summary.get("error"), "controller_pid": controller.pid,
                "scope": "Recorder-owned process interruption, never a successful trial"})
            controller.send_signal(signal.SIGINT)
            try:
                controller.wait(timeout=300)
            except subprocess.TimeoutExpired:
                stop_process(controller, group=True)
        if capture is not None and capture.poll() is None:
            capture.send_signal(signal.SIGINT)
            try:
                capture.wait(timeout=30)
            except subprocess.TimeoutExpired:
                stop_process(capture, group=True)
        if scene is not None:
            stop_process(scene, group=True)
        summary["trial_state"] = "finished"
        summary["capture_wall_duration_s"] = time.monotonic() - capture_start if capture_start else 0
        changed = [str(p) for p in sources
                   if hashlib.sha256(p.read_bytes()).hexdigest() != summary["source_sha256"][str(p)]]
        summary["changed_sources"] = changed
        if changed or capture is None or capture.returncode not in (0, 255):
            summary["success"] = False
        raw = args.output / "raw.mkv"
        if raw.exists() and raw.stat().st_size:
            try:
                probe = subprocess.run(
                    ["ffprobe", "-v", "error", "-show_entries",
                     "format=duration:stream=width,height,r_frame_rate", "-of", "json", str(raw)],
                    capture_output=True, text=True, timeout=30)
                summary["capture_probe_exit_code"] = probe.returncode
                if probe.returncode:
                    raise RuntimeError(probe.stderr)
                metadata = json.loads(probe.stdout)
                summary["capture_probe"] = metadata
                if not metadata.get("streams") or float(metadata["format"]["duration"]) <= 0:
                    raise RuntimeError("Recording has no positive-duration video stream")
            except Exception as error:
                summary.update(success=False, capture_probe_error=str(error))
        else:
            summary.update(success=False, capture_probe_error="No nonempty recording file")
        if controller is not None:
            summary["controller_exit_code"] = controller.returncode
        write_json_atomic(args.output / "recording.json", summary)
    print(json.dumps(summary, indent=2), flush=True)
    if not summary["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
