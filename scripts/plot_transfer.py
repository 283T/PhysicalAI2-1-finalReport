"""Plot an offline coordinated tool route; no simulated motion is implied."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
from plan_reach import load_arms


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--models", type=Path, default=Path("outputs/melfa_worlds"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = json.loads(args.report.read_text())
    route = report["attempts"][report["selected_attempt"]]
    board = json.loads((args.source / "board.json").read_text())
    arms = load_arms(args.source / "world.sdf", args.models)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8), layout="constrained")
    ox, oy, oz = board["origin_m"]
    width, height = np.asarray(board["size_mm"]) / 1000
    axes[0].add_patch(
        Rectangle((ox, oy), width, height, color="green", alpha=0.15, label="Board footprint")
    )
    axes[1].plot([ox, ox + width], [oz, oz], color="green", label="Receptacle rim height")
    for name, color in (("left_arm", "tab:blue"), ("right_arm", "tab:orange")):
        points = np.array(
            [
                arms[name].forward(q)[:3, 3]
                for phase in route["phases"]
                for q in phase["joint_paths"][name]
            ]
        )
        for axis, dimensions in zip(axes, ((0, 1), (0, 2))):
            x, y = dimensions
            axis.plot(points[:, x], points[:, y], color=color, label=name + " TCP plan")
            axis.scatter(*points[0, [x, y]], color=color, marker="o", s=30)
            axis.scatter(*points[-1, [x, y]], color=color, marker="x", s=50)
        base = arms[name].base[:3, 3]
        axes[0].scatter(*base[:2], marker="s", color=color, s=50)
        axes[0].text(base[0], base[1] + 0.03, name, ha="center", fontsize=9)
    for axis, label in zip(axes, ("World Y (m)", "World Z (m)")):
        axis.set_xlabel("World X (m)")
        axis.set_ylabel(label)
        axis.set_aspect("equal", adjustable="datalim")
        axis.grid(alpha=0.2)
        axis.legend(fontsize=8, loc="best")
    orbit = route["support_orbit_rad"]
    policy = (
        "rigid endpoint frames"
        if orbit is None
        else f"support orbit {np.degrees(orbit):g} deg; endpoint distance preserved"
    )
    descent = route["descent"]
    status = (
        "Descent screen passed" if descent["success"] else "Descent rejected by geometric preflight"
    )
    fig.suptitle(
        f"Offline board approach: {policy}\n{status}; physical transfer untested", fontsize=11
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
