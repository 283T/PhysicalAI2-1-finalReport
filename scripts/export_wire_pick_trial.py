"""Archive a finished MELFA wire trial, including failed/incomplete observations."""

import argparse
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np
from evidence_stream import split_evidence


def export(case, pickup, models, output):
    report = json.loads((pickup / "validation.json").read_text())
    if report.get("trial_state") != "finished":
        raise ValueError("Cannot archive a live trial")
    if report["world_sha256"] != hashlib.sha256((case / "world.sdf").read_bytes()).hexdigest():
        raise ValueError("World changed since execution")
    for name, digest in report["model_sha256"].items():
        if digest != hashlib.sha256((models / (name + ".urdf")).read_bytes()).hexdigest():
            raise ValueError("Robot model changed since execution")
    if output.exists() and any(output.iterdir()):
        raise ValueError("Use a new archive directory; do not mix trials")
    for source, directory in ((case, "case"), (pickup, "pickup")):
        for path in sorted(source.rglob("*")):
            if not path.is_file() or path.suffix not in {
                ".json",
                ".sdf",
                ".png",
                ".gz",
                ".log",
                ".npy",
            }:
                continue
            if directory == "case" and path.suffix == ".log":
                continue
            target = output / directory / path.relative_to(source)
            target.parent.mkdir(parents=True, exist_ok=True)
            if path.suffix == ".npy":
                np.savez_compressed(target.with_suffix(".npz"), depth=np.load(path))
            else:
                shutil.copyfile(path, target)
    for name in report["model_sha256"]:
        target = output / "models" / (name + ".urdf")
        target.parent.mkdir(exist_ok=True)
        shutil.copyfile(models / target.name, target)
    for path in output.rglob("*.jsonl.gz"):
        split_evidence(path)
    manifest = {
        str(path.relative_to(output)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(output.rglob("*"))
        if path.is_file() and path != output / "manifest.json"
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser()
    for name in ("case", "pickup", "models", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    export(args.case, args.pickup, args.models, args.output)


if __name__ == "__main__":
    main()
