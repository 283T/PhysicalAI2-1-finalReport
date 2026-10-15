"""Archive contact-fixture trials losslessly, with compressed full trajectories."""

import argparse
import gzip
import hashlib
import json
import shutil
from pathlib import Path


def export(source, output):
    report = json.loads((source / "validation.json").read_text())
    if hashlib.sha256((source / "world.sdf").read_bytes()).hexdigest() != report["world_sha256"]:
        raise ValueError("World changed since trial")
    trajectory = (source / "trajectory.json").read_bytes()
    json.loads(trajectory)  # Reject incomplete JSON before archiving.
    output.mkdir(parents=True, exist_ok=True)
    for name in ("world.sdf", "gripper.urdf", "validation.json"):
        shutil.copyfile(source / name, output / name)
    (output / "trajectory.json.gz").write_bytes(gzip.compress(trajectory, mtime=0))
    manifest = {
        path.name: hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(output.iterdir())
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
