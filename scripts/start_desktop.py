"""Supervise a local-only noVNC desktop for the Gazebo GUI."""

import argparse
import signal
import subprocess
import time
from pathlib import Path

from processes import stop_process


def wait_for_display(timeout=15):
    """Wait for the X server without starting a second desktop session."""
    deadline = time.monotonic() + timeout
    while subprocess.run(
        ["xdpyinfo"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    ).returncode:
        if time.monotonic() > deadline:
            raise RuntimeError("X display did not start")
        time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wait-only", action="store_true", help="Wait for the existing X display")
    args = parser.parse_args()
    if args.wait_only:
        wait_for_display()
        return
    output = Path("outputs")
    output.mkdir(exist_ok=True)
    processes = []

    def stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    with (output / "desktop.log").open("w") as log:
        try:

            def launch(command):
                process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
                processes.append(process)
                return process

            launch(["Xvfb", ":99", "-screen", "0", "1280x900x24", "-nolisten", "tcp"])
            wait_for_display()
            launch(["openbox"])
            launch(
                [
                    "x11vnc",
                    "-display",
                    ":99",
                    "-rfbport",
                    "5900",
                    "-localhost",
                    "-nopw",
                    "-forever",
                    "-shared",
                    "-noxdamage",
                ]
            )
            launch(["websockify", "--web=/usr/share/novnc", "6080", "localhost:5900"])
            launch(["xterm", "-geometry", "100x14+10+650", "-T", "PCB demo terminal", "-e", "bash"])
            print("Gazebo desktop ready on http://localhost:6080/vnc.html", flush=True)
            while True:
                if any(process.poll() is not None for process in processes):
                    raise RuntimeError("Desktop process exited; see outputs/desktop.log")
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        finally:
            for process in reversed(processes):
                stop_process(process, group=True, timeout=3)


if __name__ == "__main__":
    main()
