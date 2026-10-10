"""Remove known obsolete local experiments; preview by default."""

import argparse
import shutil
from pathlib import Path

from project_paths import ROOT

# These trials were superseded by docs/evaluation/perception_seed{1,2}.
# Keep model installs/caches, recordings and the running perception_demo_seed2 scene.
OBSOLETE = (
    "log",
    "outputs/blocked.sdf",
    "outputs/perception_case",
    "outputs/perception_case1",
    "outputs/perception_eval_seed1",
    "outputs/perception_eval_seed2",
    "outputs/perception_seed2",
    "outputs/perception_initial_camera.png",
    "outputs/perception_gazebo.log",
    "outputs/melfa_gui.config",
)


def candidates(root=ROOT):
    root = Path(root).resolve()
    paths = [root / name for name in OBSOLETE]
    for parent in ("scripts", "tests"):
        paths.extend((root / parent).rglob("__pycache__"))
    return [path for path in paths if path.exists() or path.is_symlink()]


def clean(root=ROOT, *, apply=False):
    root = Path(root).resolve()
    for path in candidates(root):
        # Unlink symbolic links without following them, including links in directories.
        if not path.parent.resolve().is_relative_to(root):
            raise ValueError("Cleanup path escapes the workspace: " + str(path))
        print(("Remove: " if apply else "Would remove: ") + str(path.relative_to(root)))
        if apply:
            if path.is_symlink() or path.is_file():
                path.unlink()
            else:
                shutil.rmtree(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--apply", action="store_true", help="Remove only the listed obsolete generated files"
    )
    args = parser.parse_args()
    clean(apply=args.apply)


if __name__ == "__main__":
    main()
