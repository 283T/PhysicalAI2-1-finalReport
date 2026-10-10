"""Export a compact comparison of physical grip and frictionless negative control."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def export(source, destination):
    destination.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for ax, name in zip(axes, ("positive", "negative")):
        folder = source / f"contact_grasp_{name}"
        rows = json.loads((folder / "trajectory.json").read_text())
        report = json.loads((folder / "validation.json").read_text())
        (destination / f"{name}_validation.json").write_text(json.dumps(report, indent=2) + "\n")
        start = rows[0]["time_s"]
        times = [r["time_s"] - start for r in rows]
        ax.plot(
            times, [r["plug"][2] * 1000 for r in rows], label="Connector center", color="#c7493a"
        )
        ax.plot(
            times,
            [r["tcp"][2] * 1000 for r in rows],
            label="Finger-tip TCP",
            color="#2779ae",
            linestyle="--",
        )
        ax.set_ylabel("World height (mm)")
        ax.set_title(
            f"Friction coefficient = {report['friction_mu']:g} | holding: {report['hold']['success']}",
            loc="left",
        )
        ax.grid(alpha=0.25)
        ax.legend(loc="center left")
        phases = {}
        for row, t in zip(rows, times):
            phases.setdefault(row["phase"], t)
        for phase, t in phases.items():
            ax.axvline(t, color="grey", alpha=0.25)
            ax.text(t + 0.1, 0.97, phase, transform=ax.get_xaxis_transform(), fontsize=8, va="top")
        # Preserve at most 10 Hz in the reviewable CSV; full traces stay in outputs/.
        with (destination / f"{name}_trajectory.csv").open("w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(
                [
                    "time_s",
                    "phase",
                    "plug_x_m",
                    "plug_y_m",
                    "plug_z_m",
                    "tcp_x_m",
                    "tcp_y_m",
                    "tcp_z_m",
                    "left_travel_m",
                    "right_travel_m",
                    "cable_z_span_m",
                ]
            )
            previous = -1.0
            for row, t in zip(rows, times):
                if t - previous >= 0.1:
                    writer.writerow(
                        [
                            t,
                            row["phase"],
                            *row["plug"],
                            *row["tcp"],
                            *row["fingers"],
                            row["cable_z_span_m"],
                        ]
                    )
                    previous = t
    axes[-1].set_xlabel("Wall time since first observation (s)")
    fig.tight_layout()
    fig.savefig(destination / "comparison.png", dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=Path("outputs"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    export(args.source, args.output)


if __name__ == "__main__":
    main()
