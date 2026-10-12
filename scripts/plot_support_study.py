"""Plot full-cycle connector pose and independent cable support observations."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from arm_kinematics import rotation_error


def plot(positive, fixed, negative, output):
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, layout="constrained")
    for name, source, color in (
        ("moving support", positive, "tab:blue"),
        ("fixed support", fixed, "tab:red"),
        ("zero friction", negative, "tab:orange"),
    ):
        rows = json.loads((source / "trajectory.json").read_text())
        times = [row["time_s"] - rows[0]["time_s"] for row in rows]
        axes[0].plot(times, [row["plug"][2] * 1000 for row in rows], color=color, label=name)
        angles = [
            np.degrees(
                np.linalg.norm(rotation_error(np.asarray(row["plug_rotation_world"]), np.eye(3)))
            )
            for row in rows
        ]
        axes[1].plot(times, angles, color=color, label=name)
        if "support_height_m" in rows[0]:
            axes[2].plot(
                times,
                [row["support_height_m"] * 1000 for row in rows],
                color=color,
                label=name + " actual",
            )
            axes[2].plot(
                times,
                [row["support_command_m"] * 1000 for row in rows],
                color=color,
                linestyle="--",
                alpha=0.5,
                label=name + " command",
            )
        if source == positive:
            phases = {}
            for row, time in zip(rows, times):
                phases.setdefault(row["phase"], time)
            for phase in ("lift", "hold", "return_to_nest", "release", "retract"):
                if phase not in phases:
                    continue
                time = phases[phase]
                for axis in axes:
                    axis.axvline(time, color="grey", alpha=0.15)
                axes[0].text(
                    time,
                    1.01,
                    phase.replace("return_to_nest", "return"),
                    transform=axes[0].get_xaxis_transform(),
                    fontsize=8,
                    rotation=25,
                )
    axes[0].axhline(230, color="grey", linestyle="--", label="nest seat")
    axes[1].axhline(np.degrees(0.05), color="grey", linestyle="--", label="seat angle limit")
    axes[0].set_ylabel("Connector world Z (mm)")
    axes[1].set_ylabel("Angle from seated pose (deg)")
    axes[2].set_ylabel("Support displacement (mm)")
    axes[2].set_xlabel("Wall time since trial start (s)")
    for axis in axes:
        axis.grid(alpha=0.2)
        axis.legend(fontsize=8)
    fig.suptitle(
        "Moving vs fixed cable support\nContact fixtures, one connector and articulated cable",
        fontsize=12,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    for name in ("positive", "fixed", "negative", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    plot(args.positive, args.fixed, args.negative, args.output)


if __name__ == "__main__":
    main()
