"""Create a camera-resolution comparison without changing physical scene inputs."""

import argparse
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from evidence_io import write_json_atomic
from inspection_camera import validate_calibration


def resize(case, calibration_path, output, scale):
    if type(scale) is not int or not 2 <= scale <= 4:
        raise ValueError("Integer resolution scale 2..4 required")
    if output.resolve().is_relative_to(case.resolve()):
        raise ValueError("Output must be separate from source")
    if output.exists() and any(output.iterdir()):
        raise ValueError("Use a new empty directory")
    calibration = validate_calibration(json.loads(calibration_path.read_text()))
    tree = ET.parse(case / "world.sdf")
    matching = [
        s
        for s in tree.findall("world/model/link/sensor[@type='rgbd_camera']")
        if s.findtext("topic") == calibration["topic"]
    ]
    if len(matching) != 1:
        raise ValueError("Exactly one matching RGB-D camera required")
    image = matching[0].find("camera/image")
    if any(int(image.findtext(k)) != calibration[k] for k in ("width", "height")):
        raise ValueError("Source camera resolution and calibration disagree")
    previous = {k: calibration[k] for k in ("width", "height")}
    for key in ("width", "height"):
        calibration[key] *= scale
        image.find(key).text = str(calibration[key])
    shutil.copytree(case, output, dirs_exist_ok=True)
    ET.indent(tree)
    tree.write(output / "world.sdf", encoding="utf-8", xml_declaration=True)
    write_json_atomic(output / "camera_calibration.json", calibration)
    write_json_atomic(
        output / "camera_resolution.json",
        {
            "source_world_sha256": hashlib.sha256((case / "world.sdf").read_bytes()).hexdigest(),
            "source_calibration_sha256": hashlib.sha256(calibration_path.read_bytes()).hexdigest(),
            "old_resolution": previous,
            "new_resolution": {k: calibration[k] for k in previous},
            "scope": "Only matching camera image width/height and corresponding calibration dimensions changed. No physical model, control or acceptance changes; no perception success evidence",
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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("case", "calibration", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--scale", type=int, default=2)
    args = parser.parse_args()
    resize(args.case, args.calibration, args.output, args.scale)
