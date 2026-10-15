"""Copy a case with a finer physics timestep; preserve geometry and acceptance."""

import argparse
import hashlib
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from evidence_io import write_json_atomic


def refine(case, output, timestep):
    case, output = Path(case), Path(output)
    if output.resolve().is_relative_to(case.resolve()):
        raise ValueError("Output must be separate from source")
    if output.exists() and any(output.iterdir()):
        raise ValueError("Use a new empty output directory")
    source = case / "world.sdf"
    tree = ET.parse(source)
    steps = tree.findall("world/physics/max_step_size")
    if len(steps) != 1:
        raise ValueError("Exactly one explicit physics timestep required")
    original = float(steps[0].text)
    if not math.isfinite(timestep) or not 0 < timestep < original:
        raise ValueError("New timestep must be positive, finite and strictly finer")
    steps[0].text = str(timestep)
    shutil.copytree(case, output, dirs_exist_ok=True)
    ET.indent(tree)
    tree.write(output / "world.sdf", encoding="utf-8", xml_declaration=True)
    write_json_atomic(
        output / "physics_refinement.json",
        {
            "source_world_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "original_max_step_size_s": original,
            "max_step_size_s": timestep,
            "scope": "Only max_step_size changed; all geometry, controller gains, force caps and task acceptance preserved",
        },
    )
    write_json_atomic(
        output / "manifest.json",
        {
            str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(output.rglob("*"))
            if p.is_file() and p.name != "manifest.json"
        },
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timestep", type=float, required=True)
    args = parser.parse_args()
    refine(args.case, args.output, args.timestep)
