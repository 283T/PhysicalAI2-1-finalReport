"""Copy a case and add a calibrated view without changing its physical task."""

import argparse
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from evidence_io import write_json_atomic
from inspection_camera import add_inspection_camera


def add_camera(case, calibration, name, output):
    if output.exists() and any(output.iterdir()):
        raise ValueError("Use an empty new case directory")
    data = json.loads(calibration.read_text())
    tree = ET.parse(case / "world.sdf")
    add_inspection_camera(tree.getroot().find("world"), data, model_name=name)
    shutil.copytree(case, output, dirs_exist_ok=True)
    ET.indent(tree)
    tree.write(output / "world.sdf", encoding="utf-8", xml_declaration=True)
    write_json_atomic(output / (name + ".json"), data)
    write_json_atomic(
        output / "camera_addition.json",
        {
            "source_world_sha256": hashlib.sha256((case / "world.sdf").read_bytes()).hexdigest(),
            "calibration_sha256": hashlib.sha256(calibration.read_bytes()).hexdigest(),
            "camera_model": name,
            "scope": "Added static RGB-D view only; drawing, board, robot and harness geometry unchanged",
        },
    )
    write_json_atomic(
        output / "manifest.json",
        {
            str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(output.rglob("*"))
            if p.is_file() and p != output / "manifest.json"
        },
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("case", "calibration", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--name", default="nest_camera")
    args = parser.parse_args()
    add_camera(args.case, args.calibration, args.name, args.output)


if __name__ == "__main__":
    main()
