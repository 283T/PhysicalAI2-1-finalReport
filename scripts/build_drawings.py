"""Create original, dimensioned assembly drawings and expected task fixtures."""

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def main():
    root = Path(__file__).resolve().parents[1]
    board = json.loads((root / "configs/board.json").read_text())
    output = root / "drawings"
    output.mkdir(exist_ok=True)
    for variant in ("a", "b"):
        task = json.loads((root / f"examples/task_{variant}.json").read_text())
        fig, ax = plt.subplots(figsize=(10, 8), dpi=160)
        width, height = board["size_mm"]
        ax.add_patch(Rectangle((0, 0), width, height, facecolor="#e8eef5", edgecolor="#193e2a", lw=2))
        for socket in board["sockets"]:
            x, y = socket["x_mm"], socket["y_mm"]
            ax.add_patch(Rectangle((x-30, y-20), 60, 40, facecolor="#a6c0d9", edgecolor="#193e2a"))
            for offset in (-10, 10):
                ax.add_patch(Rectangle((x+offset-4, y-4), 8, 8, facecolor="white", edgecolor="#193e2a"))
            ax.text(x, y+12, socket["id"], ha="center", fontsize=12)
        x, y = task["x_mm"], task["y_mm"]
        ax.add_patch(Rectangle((x-20, y-10), 40, 20, fill=False, edgecolor="#c65b00", lw=3))
        ax.annotate("", xy=(x+17, y), xytext=(x-12, y), arrowprops={"arrowstyle": "->", "color": "#c65b00", "lw": 2})
        for offset, color in ((-5, "#b52a28"), (5, "#293946")):
            ax.plot([x+offset, x+offset, x+offset+12], [y-10, y-32, y-37], color=color, lw=3)
        ax.text(x+24, y-28, "WH1 / P1", fontsize=10)
        ax.plot([0, x], [y, y], "--", color="#555", lw=1)
        ax.plot([x, x], [0, y], "--", color="#555", lw=1)
        ax.annotate("", xy=(x, -12), xytext=(0, -12), arrowprops={"arrowstyle": "<->"})
        ax.text(x/2, -19, f"X = {x} mm", ha="center", fontsize=12)
        ax.annotate("", xy=(-12, y), xytext=(-12, 0), arrowprops={"arrowstyle": "<->"})
        ax.text(-19, y/2, f"Y = {y} mm", ha="center", va="center", rotation=90, fontsize=12)
        ax.scatter([0], [0], color="#193e2a")
        ax.text(2, 3, "ORIGIN (0, 0)", fontsize=9)
        ax.annotate("+X", xy=(35, 0), xytext=(7, -4), fontsize=9)
        ax.annotate("+Y", xy=(0, 35), xytext=(2, 24), fontsize=9)
        ax.text(0, height+18, f"HARNESS CONNECTION {variant.upper()} | TOP VIEW", weight="bold", fontsize=16)
        ax.text(0, height+7, f"PANEL: {width} x {height} mm | Units: mm | NOT TO SCALE", fontsize=10)
        ax.text(0, -28, f"CONNECT: 1 x WH1 / P1 TO {task['socket']}\n"
                f"CENTER: X={x} mm, Y={y} mm | YAW: 0 deg (+X)\n"
                "WH1: 2-wire harness. P1: 2-pin plug.\n"
                "Unused socket: leave empty. Cable: schematic.", fontsize=10, linespacing=1.5, va="top")
        ax.set_xlim(-30, width+10)
        ax.set_ylim(-65, height+30)
        ax.set_aspect("equal")
        ax.axis("off")
        fig.tight_layout()
        for extension in ("png", "svg"):
            path = output / f"assembly_{variant}.{extension}"
            fig.savefig(path, facecolor="white")
            if extension == "svg":
                path.write_text("\n".join(line.rstrip() for line in path.read_text().splitlines()) + "\n")
        plt.close(fig)


if __name__ == "__main__":
    main()
