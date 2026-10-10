"""Review measured connector/TCP motion and finger feedback after pickup approach."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def export(source, output):
    output.mkdir(parents=True, exist_ok=True)
    rows = json.loads((source / "trajectory.json").read_text())
    rows = [row for row in rows if row["phase"] in ("approach", "close", "lift", "hold", "release")]
    if not rows:
        raise ValueError("No pickup approach trajectory")
    start = rows[0]["sim_time_s"]
    times = [row["sim_time_s"] - start for row in rows]
    reference_row = next((r for r in reversed(rows) if r["phase"] == "close"), None)
    if reference_row is None:
        reference_row = rows[0]
    reference = np.array(reference_row["plug"]) - reference_row["tcp"]
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(
        times, [r["plug"][2] * 1000 for r in rows], label="Connector center", color="#c7493a"
    )
    axes[0].plot(
        times, [r["tcp"][2] * 1000 for r in rows], label="TCP", color="#2779ae", linestyle="--"
    )
    axes[0].set_ylabel("World height (mm)")
    axes[0].legend(loc="center left")
    for i, label in enumerate(("Left finger", "Right finger")):
        axes[1].plot(times, [r["fingers"][i] * 1000 for r in rows], label=label)
    axes[1].set_ylabel("Finger travel (mm)")
    axes[1].legend(loc="center left")
    selected = [(t, r) for t, r in zip(times, rows) if r["phase"] in ("lift", "hold")]
    axes[2].plot(
        [t for t, _ in selected],
        [np.linalg.norm(np.array(r["plug"]) - r["tcp"] - reference) * 1000 for _, r in selected],
        color="#7758a7",
    )
    axes[2].axhline(5, color="grey", linestyle="--", label="5 mm acceptance limit")
    axes[2].set_ylabel("Relative shift (mm)")
    axes[2].legend(loc="center left")
    phases = {}
    for t, row in zip(times, rows):
        phases.setdefault(row["phase"], t)
    for ax in axes:
        ax.grid(alpha=0.25)
        for phase, t in phases.items():
            ax.axvline(t, color="grey", alpha=0.25)
            ax.text(
                t + 0.1, 1.01, phase, transform=ax.get_xaxis_transform(), va="bottom", fontsize=8
            )
    axes[-1].set_xlabel("Simulated time since approach began (s)")
    fig.suptitle("MELFA pickup: measured motion (approach onward)")
    fig.tight_layout()
    fig.savefig(output / "pickup.png", dpi=160)
    plt.close(fig)
    with (output / "trajectory.csv").open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "sim_time_s",
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
        previous = -1
        for t, row in zip(times, rows):
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    export(args.source, args.output)


if __name__ == "__main__":
    main()
