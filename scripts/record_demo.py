"""Record actual Gazebo execution with input drawing and local AI output alongside."""

import json
from pathlib import Path
import subprocess

from read_drawing import ROOT, read_drawing


PREFIX = ["docker", "compose", "--profile", "gui"]


def call(command, **kwargs):
    return subprocess.run(command, cwd=ROOT, check=True, **kwargs)


def main():
    output = ROOT / "outputs"
    output.mkdir(exist_ok=True)
    recording = ROOT / "recordings"
    recording.mkdir(exist_ok=True)
    call(PREFIX + ["up", "-d", "gui"])
    for variant in ("a", "b"):
        print(f"Reading and recording drawing {variant.upper()}", flush=True)
        task_path = output / f"record_task_{variant}.json"
        task = read_drawing(ROOT / f"drawings/assembly_{variant}.png", task_path)
        caption = output / f"record_caption_{variant}.txt"
        details = (f"DRAWING {task['drawing_id']} | LOCAL AI OUTPUT\n"
                   f"Qwen3-VL 2B / Ollama\n"
                   f"Part: {task['component']} | Socket: {task['socket']}\n"
                   f"X: {task['x_mm']} mm | Y: {task['y_mm']} mm\n"
                   f"Yaw: {task['yaw_deg']} deg | Quantity: 1\n"
                   "Validated before robot execution\n"
                   "Actual Gazebo / ROS 2 | 1x speed\n"
                   "Ideal grasp / enlarged pins and holes\n")
        caption.write_text(details + "Status: starting simulation\n")
        call(PREFIX + ["restart", "gui"])
        command = PREFIX + ["exec", "-T", "gui", "/ros_entrypoint.sh", "python3",
                            "scripts/run_insertion.py", "--task", f"outputs/{task_path.name}",
                            "--gui", "--start-delay", "15", "--keep-open"]
        robot = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True, bufsize=1)
        capture = None
        try:
            # X display readiness is handled by the desktop supervisor. Wait until it is ready.
            call(PREFIX + ["exec", "-T", "gui", "python3", "-c",
                 "import subprocess,time; "
                 "exec('for i in range(100):\\n if subprocess.run([\"xdpyinfo\"],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0: break\\n time.sleep(.1)\\nelse: raise RuntimeError(\"Display unavailable\")')"])
            filters = ("[0:v]pad=1920:900:640:0:color=0x142c40[bg];"
                       "[1:v]scale=640:512[draw];[bg][draw]overlay=0:0,"
                       "drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:"
                       f"textfile=/workspace/outputs/{caption.name}:reload=1:"
                       "fontcolor=white:fontsize=20:x=22:y=530:line_spacing=10[v]")
            capture_command = PREFIX + ["exec", "-T", "gui", "ffmpeg", "-y", "-loglevel", "error",
                "-f", "x11grab", "-video_size", "1280x900", "-framerate", "15", "-i", ":99",
                "-loop", "1", "-i", f"/workspace/drawings/assembly_{variant}.png",
                "-filter_complex", filters, "-map", "[v]", "-t", "60", "-an",
                "-c:v", "libx264", "-preset", "ultrafast", "-crf", "24", "-pix_fmt", "yuv420p",
                "-movflags", "+faststart", f"/workspace/recordings/drawing_{variant}.mp4"]
            capture = subprocess.Popen(capture_command, cwd=ROOT)
            success = False
            with (output / f"record_robot_{variant}.log").open("w") as log:
                for line in robot.stdout:
                    print(line, end="", flush=True)
                    log.write(line)
                    if line[:1] in "123456" and line[1:2] == ".":
                        caption.write_text(details + f"Status: {line.strip()}\n")
                    if line.startswith("Demo complete."):
                        if capture.poll() is not None:
                            raise RuntimeError("Recording ended before insertion completed")
                        result = json.loads((output / "insertion_result.json").read_text())
                        success = result.get("success") and result.get("drawing_id") == task["drawing_id"]
                        caption.write_text(details + ("INSERTION SUCCESS | released\n" if success else "INSERTION FAILED\n"))
                        (output / f"record_result_{variant}.json").write_text(json.dumps(result, indent=2)+"\n")
                        break
            if not success:
                raise RuntimeError("Insertion did not succeed; recording is not a successful demo")
            if capture.wait(timeout=60):
                raise RuntimeError("Video capture failed")
        finally:
            if capture and capture.poll() is None:
                capture.terminate()
                capture.wait(timeout=10)
            if robot.poll() is None:
                robot.terminate()
                robot.wait(timeout=10)
    concat = recording / "concat.txt"
    concat.write_text("file 'drawing_a.mp4'\nfile 'drawing_b.mp4'\n")
    call(PREFIX + ["exec", "-T", "gui", "ffmpeg", "-y", "-loglevel", "error",
                  "-f", "concat", "-safe", "0", "-i", "/workspace/recordings/concat.txt",
                  "-c", "copy", "-movflags", "+faststart", "/workspace/recordings/pcb_assembly_demo.mp4"])
    print("Recorded recordings/pcb_assembly_demo.mp4 (A then B, 1x speed)", flush=True)


if __name__ == "__main__":
    main()
