"""Export physical top-grip/return traces and rotation-aware holding evidence."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from arm_kinematics import rotation_error


def export(sources, output):
    output.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, layout="constrained")
    for name, source, color in sources:
        rows = json.loads((source / "trajectory.json").read_text())
        report = json.loads((source / "validation.json").read_text())
        (output / (name + "_validation.json")).write_text(json.dumps(report, indent=2) + "\n")
        start = rows[0]["time_s"]
        times = [row["time_s"] - start for row in rows]
        closed = next(row for row in reversed(rows) if row["phase"] == "close")
        reference = np.asarray(closed["plug"]) - closed["tcp"]
        rotations = [
            float(
                np.linalg.norm(
                    rotation_error(
                        np.asarray(row["plug_rotation_world"]),
                        np.asarray(closed["plug_rotation_world"]),
                    )
                )
            )
            for row in rows
        ]
        shifts = [
            float(np.linalg.norm(np.asarray(row["plug"]) - row["tcp"] - reference)) for row in rows
        ]
        axes[0].plot(
            times, [row["plug"][2] * 1000 for row in rows], color=color, label=name + " connector"
        )
        axes[0].plot(
            times,
            [row["tcp"][2] * 1000 for row in rows],
            color=color,
            linestyle="--",
            alpha=0.5,
            label=name + " TCP",
        )
        holding = [i for i, row in enumerate(rows) if row["phase"] in ("lift", "hold")]
        axes[1].plot(
            [times[i] for i in holding],
            [shifts[i] * 1000 for i in holding],
            color=color,
            label=name,
        )
        axes[2].plot(
            [times[i] for i in holding],
            [np.degrees(rotations[i]) for i in holding],
            color=color,
            label=name,
        )
        phases = {}
        for time, row in zip(times, rows):
            phases.setdefault(row["phase"], time)
        if name == "positive":
            for axis in axes:
                for phase, time in phases.items():
                    axis.axvline(time, color="grey", alpha=0.15)
            for phase, time in phases.items():
                axes[0].text(
                    time + 0.1,
                    1.02,
                    phase.replace("return_to_nest", "return"),
                    transform=axes[0].get_xaxis_transform(),
                    fontsize=8,
                )
        with (output / (name + "_trajectory.csv")).open("w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(
                [
                    "wall_time_s",
                    "phase",
                    "plug_x_m",
                    "plug_y_m",
                    "plug_z_m",
                    "tcp_x_m",
                    "tcp_y_m",
                    "tcp_z_m",
                    "left_finger_m",
                    "right_finger_m",
                    "cable_z_span_m",
                    "relative_shift_m",
                    "rotation_change_rad",
                ]
            )
            previous = -1.0
            for i, row in enumerate(rows):
                if times[i] - previous >= 0.1:
                    writer.writerow(
                        [
                            times[i],
                            row["phase"],
                            *row["plug"],
                            *row["tcp"],
                            *row["fingers"],
                            row["cable_z_span_m"],
                            shifts[i],
                            rotations[i],
                        ]
                    )
                    previous = times[i]
    axes[0].set_ylabel("World height (mm)")
    axes[1].set_ylabel("Relative shift (mm)")
    axes[2].set_ylabel("Rotation change (deg)")
    axes[1].axhline(5, color="grey", linestyle="--", label="5 mm hold limit")
    axes[2].axhline(np.degrees(0.05), color="grey", linestyle="--", label="0.05 rad hold limit")
    for axis in axes:
        axis.grid(alpha=0.2)
        axis.legend(fontsize=8, loc="center left")
    axes[-1].set_xlabel("Wall time since first observation (s)")
    fig.suptitle(
        "Contact top pickup and return: passive nest + cable tray\nCartesian carriage, not MELFA regrasp or board insertion",
        fontsize=11,
    )
    fig.savefig(output / "comparison.png", dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--positive", type=Path, required=True)
    parser.add_argument("--negative", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    export(
        (("positive", args.positive, "tab:blue"), ("negative", args.negative, "tab:orange")),
        args.output,
    )


if __name__ == "__main__":
    main()
