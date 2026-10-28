"""Acquire new RGB-D while the owning arm session maintains its commands."""

import json
import math
import subprocess
import time

from processes import stop_process


def acquire_observation(session, output, *, wire_distance_mm=None, calibration=None):
    timeout = float(getattr(session, "observation_timeout_wall_s", 90))
    if not math.isfinite(timeout) or not 5 <= timeout <= 300:
        raise ValueError("Observation wall timeout must be finite and within 5 to 300 seconds")
    output.mkdir(parents=True, exist_ok=True)
    command = [
        "python3",
        "scripts/observe_connectors.py",
        "--bridge",
        "--plugs",
        "P3",
        "--require-6d",
        "--output",
        str(output),
        "--timeout",
        str(timeout),
    ]
    if wire_distance_mm is not None:
        command += ["--wire-supports", "--wire-distance-mm", str(wire_distance_mm)]
    if calibration:
        command += ["--calibration", str(calibration), "--end-markers"]
    with (output / "run.log").open("w") as log:
        process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
        try:
            deadline = time.monotonic() + timeout + 30
            while process.poll() is None:
                if time.monotonic() > deadline:
                    raise RuntimeError("Fresh observation timed out")
                session.tick()
            if process.returncode:
                raise RuntimeError("Fresh observation rejected; see " + str(output))
            return json.loads(
                (
                    output
                    / ("wire_supports.json" if wire_distance_mm is not None else "detections.json")
                ).read_text()
            )
        finally:
            stop_process(process, group=True)
