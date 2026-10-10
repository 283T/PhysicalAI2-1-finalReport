"""Move only the Gazebo GUI camera; the calibrated RGB-D sensor is unchanged."""

import math
import subprocess


def main():
    pitch, yaw = 0.54, -0.644
    # Quaternion for zero roll, looking from the pallet side towards the arms.
    x = -math.sin(pitch / 2) * math.sin(yaw / 2)
    y = math.sin(pitch / 2) * math.cos(yaw / 2)
    z = math.cos(pitch / 2) * math.sin(yaw / 2)
    w = math.cos(pitch / 2) * math.cos(yaw / 2)
    request = (
        f"pose {{ position {{ x: -1.4 y: 0.9 z: 1.35 }} "
        f"orientation {{ x: {x} y: {y} z: {z} w: {w} }} }}"
    )
    result = subprocess.run(
        [
            "gz",
            "service",
            "-s",
            "/gui/move_to/pose",
            "--reqtype",
            "gz.msgs.GUICamera",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            request,
        ],
        capture_output=True,
        text=True,
        check=True,
        timeout=8,
    )
    if "data: true" not in result.stdout:
        raise RuntimeError("Gazebo GUI camera did not accept the view: " + result.stdout)
    print("GUI view updated; RGB-D extrinsics unchanged")


if __name__ == "__main__":
    main()
