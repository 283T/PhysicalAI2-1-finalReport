"""Create a gain-comparison case while preserving geometry and force limits."""

import argparse
import hashlib
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from evidence_io import write_json_atomic


def retune(case, output, arm, index, p_gain, d_gain):
    if not 1 <= index <= 6 or arm not in ("left_arm", "right_arm"):
        raise ValueError("Known arm and joint index 1..6 required")
    if not all(math.isfinite(v) and v > 0 for v in (p_gain, d_gain)):
        raise ValueError("Positive finite P and D gains required")
    if output.resolve().is_relative_to(case.resolve()):
        raise ValueError("Output must be separate from the source case")
    if output.exists() and any(output.iterdir()):
        raise ValueError("Use a new empty case directory")
    tree = ET.parse(case / "world.sdf")
    model = tree.getroot().find(f"world/model[@name='{arm}']")
    matches = [
        p
        for p in model.findall("plugin")
        if p.findtext("joint_name") == f"{arm}_rv4frl_joint_{index}"
        and p.get("name") == "gz::sim::systems::JointPositionController"
    ]
    if len(matches) != 1:
        raise ValueError("Exactly one MELFA joint force-PID controller required")
    plugin = matches[0]
    if plugin.findtext("use_velocity_commands", "false") != "false":
        raise ValueError("Force-PID mode required")
    changes = {}
    for name, value in (("p_gain", p_gain), ("d_gain", d_gain)):
        element = plugin.find(name)
        changes[name] = {"old": float(element.text), "new": value}
        element.text = str(value)
    shutil.copytree(case, output, dirs_exist_ok=True)
    ET.indent(tree)
    tree.write(output / "world.sdf", encoding="utf-8", xml_declaration=True)
    write_json_atomic(
        output / "controller_tuning.json",
        {
            "source_world_sha256": hashlib.sha256((case / "world.sdf").read_bytes()).hexdigest(),
            "arm": arm,
            "joint_index": index,
            "changes": changes,
            "scope": "Only selected P/D gains changed. Geometry, initial placement, I gain, force caps and task acceptance unchanged",
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
    parser.add_argument("--case", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--arm", choices=("left_arm", "right_arm"), required=True)
    parser.add_argument("--joint-index", type=int, required=True)
    parser.add_argument("--p-gain", type=float, required=True)
    parser.add_argument("--d-gain", type=float, required=True)
    args = parser.parse_args()
    retune(args.case, args.output, args.arm, args.joint_index, args.p_gain, args.d_gain)


if __name__ == "__main__":
    main()
