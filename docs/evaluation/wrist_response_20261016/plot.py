"""Regenerate the comparison figure from preserved native joint observations."""

import gzip
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
trials = json.loads((root / "summary.json").read_text())["trials"]
figure, axes = plt.subplots(len(trials), 1, figsize=(10, 2.2 * len(trials)), sharex=True)
for axis, trial in zip(axes, trials):
    rows = json.loads(
        gzip.decompress((root / trial["archive"] / "trajectory.json.gz").read_bytes())
    )
    for arm in ("left_arm", "right_arm"):
        axis.plot(
            [r["sim_time_s"] for r in rows],
            [
                1000 * (r["arms"][arm]["q_rad"][5] - r["arms"][arm]["command_q_rad"][5])
                for r in rows
            ],
            label=arm,
            linewidth=0.8,
        )
    for gate in (-10, 10):
        axis.axhline(gate, color="black", linestyle="--", linewidth=0.6)
    axis.set_title(
        trial["archive"] + (" (step sequence passed)" if trial["success"] else " (stopped)")
    )
    axis.set_ylabel("q6 error [mrad]")
    axis.grid(alpha=0.25)
axes[0].legend(loc="upper right")
axes[-1].set_xlabel("Simulation time [s]")
figure.suptitle("Empty-gripper response: tracking evidence only, no assembly result")
figure.tight_layout(rect=(0, 0, 1, 0.97))
figure.savefig(root / "comparison.png", dpi=160)
