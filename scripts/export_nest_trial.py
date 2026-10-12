"""Preserve MELFA nest trials and full-cycle tool-relative slip measurements."""

import argparse
import csv
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np
from assembly_case import validate_connection_plan
from grasp_validation import assess_relative_grasp


def export(source, output):
    pickup = source / "pickup"
    report = json.loads((pickup / "validation.json").read_text())
    world = source / "case/world.sdf"
    if report["world_sha256"] != hashlib.sha256(world.read_bytes()).hexdigest():
        raise ValueError("Trial world changed")
    plan = json.loads((source / "connection_plan.json").read_text())
    validate_connection_plan(plan, json.loads((source / "case/board.json").read_text()))
    if (
        plan["image_sha256"]
        != hashlib.sha256((source / "case/drawing.png").read_bytes()).hexdigest()
    ):
        raise ValueError("Drawing changed since local inference")
    output.mkdir(parents=True, exist_ok=True)
    for name in ("world.sdf", "board.json", "drawing.png", "regrasp_nest.json", "case.json"):
        shutil.copyfile(source / "case" / name, output / name)
    shutil.copyfile(source / "case/manifest.json", output / "case_manifest.json")
    for name in ("connection_plan.json", "connection_plan.audit.json"):
        shutil.copyfile(source / name, output / name)
    for name in ("validation.json", "grasp_plan.json"):
        shutil.copyfile(pickup / name, output / name)
    for name in ("detections.json", "camera_intrinsics.json", "camera.png"):
        shutil.copyfile(pickup / "observations" / name, output / name)
    depth = np.load(pickup / "observations/depth.npy")
    np.savez_compressed(output / "depth.npz", depth=depth)
    rows = json.loads((pickup / "trajectory.json").read_text())
    references = next((row["arms"] for row in reversed(rows) if row["phase"] == "close"), None)
    transport_references = next(
        (row["arms"] for row in reversed(rows) if row["phase"] == "hold"), references
    )
    summary = {
        "success": report["success"],
        "error": report.get("error"),
        "grasp_reference_available": references is not None,
        "phase_endpoints": {},
        "transport_grasp_maxima": {},
    }
    with (output / "trajectory.csv").open("w", newline="") as file:
        writer = csv.writer(file, lineterminator="\n")
        writer.writerow(
            [
                "sim_time_s",
                "phase",
                "arm",
                "plug_id",
                "plug_x_m",
                "plug_y_m",
                "plug_z_m",
                "tcp_x_m",
                "tcp_y_m",
                "tcp_z_m",
                "left_travel_m",
                "right_travel_m",
                "tool_relative_shift_m",
                "tool_relative_angle_rad",
                "cable_z_span_m",
            ]
        )
        previous = -1.0
        for row in rows:
            summary["phase_endpoints"][row["phase"]] = row
            active_references = (
                transport_references if row["phase"].startswith("nest_") else references
            )
            checks = {
                name: assess_relative_grasp(active_references[name], arm)
                if active_references is not None
                else {}
                for name, arm in row["arms"].items()
            }
            if row["phase"].startswith("nest_transfer_"):
                for name, check in checks.items():
                    maximum = summary["transport_grasp_maxima"].setdefault(
                        name, {"position_shift_m": 0.0, "angle_shift_rad": 0.0}
                    )
                    for key in maximum:
                        maximum[key] = max(maximum[key], check[key])
            if row["sim_time_s"] - previous < 0.1 and row is not rows[-1]:
                continue
            for name, arm in row["arms"].items():
                check = checks[name]
                writer.writerow(
                    [
                        row["sim_time_s"],
                        row["phase"],
                        name,
                        arm["plug_id"],
                        *arm["plug"],
                        *arm["tcp"],
                        *arm["fingers"],
                        check.get("position_shift_m"),
                        check.get("angle_shift_rad"),
                        row["cable_z_span_m"],
                    ]
                )
            previous = row["sim_time_s"]
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    manifest = {
        str(path.relative_to(output)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(output.rglob("*"))
        if path.is_file() and path.name != "manifest.json"
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    export(args.source, args.output)


if __name__ == "__main__":
    main()
