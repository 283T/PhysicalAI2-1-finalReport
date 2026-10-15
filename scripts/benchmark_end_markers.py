"""Time perception on archived pixels, separately from ROS transport and rendering."""

import argparse
import hashlib
import json
import time
from pathlib import Path

import cv2
import numpy as np
from end_markers import detect_end_markers
from inspection_camera import validate_calibration


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--iterations", type=int, default=5)
    args = parser.parse_args()
    if not 1 <= args.iterations <= 100:
        parser.error("Use 1 to 100 iterations")
    files = ["camera.png", "depth.npz", "camera_intrinsics.json", "calibration.json"]
    pixels = cv2.imread(str(args.input / "camera.png"))
    if pixels is None:
        raise ValueError("Cannot decode archived camera image")
    depth = np.load(args.input / "depth.npz")["depth"]
    intrinsics = json.loads((args.input / "camera_intrinsics.json").read_text())
    calibration = validate_calibration(json.loads((args.input / "calibration.json").read_text()))
    rows = []
    for _ in range(args.iterations):
        started = time.perf_counter()
        detections, _ = detect_end_markers(pixels, depth, intrinsics, calibration)
        rows.append(
            {
                "wall_s": time.perf_counter() - started,
                "detected_plugs": [r["plug_id"] for r in detections],
            }
        )
    report = {
        "scope": "Archived end-tag detector timing only; no transport, physics or accuracy evaluation",
        "input_sha256": {
            name: hashlib.sha256((args.input / name).read_bytes()).hexdigest() for name in files
        },
        "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "opencv_version": cv2.__version__,
        "opencv_threads": cv2.getNumThreads(),
        "iterations": rows,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
