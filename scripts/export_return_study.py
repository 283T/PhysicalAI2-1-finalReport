"""Preserve return diagnostics, including trials that stopped before release."""

import argparse
import csv
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np
from arm_kinematics import rotation_error
from grasp_validation import assess_hold, assess_seat


def export(sources, output):
    output.mkdir(parents=True, exist_ok=True)
    summaries = {}
    for source in sources:
        target = output / source.name
        target.mkdir(exist_ok=True)
        rows = json.loads((source / "trajectory.json").read_text())
        report = json.loads((source / "validation.json").read_text())
        for name in ("world.sdf", "validation.json"):
            shutil.copyfile(source / name, target / name)
        phases = {row["phase"]: row for row in rows}
        held = [row for row in rows if row["phase"] == "hold"]
        seat = [row for row in rows if row["phase"] == "observe_seat"]
        summaries[source.name] = {
            "success": report["success"],
            "error": report.get("error"),
            "motion_aborted": report.get("motion_aborted"),
            "expectation_met": report.get("expectation_met"),
            "return_mode": report["return_mode"],
            "return_reference": report.get("return_reference", "command"),
            "return_target_m": report.get("return_target_m"),
            "reference_seat": report.get("reference_seat"),
            "pin_relief": report.get("pin_relief", False),
            "moving_tail_support": report.get("moving_tail_support", False),
            "matched_static_support": report.get("matched_static_support", False),
            "hold_maximum_support_error_m": max(
                (abs(row["support_error_m"]) for row in held if "support_error_m" in row),
                default=None,
            ),
            "contact_monitor_ready": report.get("contact_monitor_ready", False),
            "contact_messages": report.get("contact_messages"),
            "hold_from_trajectory": assess_hold(phases["open"], held, phases["close"]),
            "reseat_from_trajectory": assess_seat(seat),
            "phase_endpoints": phases,
        }
        with (target / "trajectory.csv").open("w", newline="") as file:
            writer = csv.writer(file, lineterminator="\n")
            writer.writerow(
                [
                    "wall_time_s",
                    "phase",
                    "plug_x_m",
                    "plug_y_m",
                    "plug_z_m",
                    "tcp_z_m",
                    "lift_command_m",
                    "lift_joint_m",
                    "lift_error_m",
                    "angle_error_rad",
                    "wire_tip_x_m",
                    "wire_tip_y_m",
                    "wire_tip_z_m",
                    "support_command_m",
                    "support_height_m",
                    "support_error_m",
                ]
            )
            start, previous = rows[0]["time_s"], -1.0
            for row in rows:
                elapsed = row["time_s"] - start
                if elapsed - previous < 0.1 and row is not rows[-1]:
                    continue
                angle = float(
                    np.linalg.norm(
                        rotation_error(np.asarray(row["plug_rotation_world"]), np.eye(3))
                    )
                )
                writer.writerow(
                    [
                        elapsed,
                        row["phase"],
                        *row["plug"],
                        row["tcp"][2],
                        row["lift_command_m"],
                        row["lift_joint_m"],
                        row["lift_error_m"],
                        angle,
                        *row["cable_links_world"]["P1_wire_11"],
                        row.get("support_command_m"),
                        row.get("support_height_m"),
                        row.get("support_error_m"),
                    ]
                )
                previous = elapsed
    (output / "summary.json").write_text(json.dumps(summaries, indent=2) + "\n")
    manifest = {
        str(path.relative_to(output)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(output.rglob("*"))
        if path.is_file() and path.name != "manifest.json"
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return summaries


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("sources", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if len({source.name for source in args.sources}) != len(args.sources):
        parser.error("Source directory names must be unique")
    export(args.sources, args.output)


if __name__ == "__main__":
    main()
