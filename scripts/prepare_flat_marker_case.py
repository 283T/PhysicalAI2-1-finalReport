"""Copy a scene and replace only visual tags with coplanar black/white cells."""

import argparse
import copy
import hashlib
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from end_markers import END_OFFSET_M, add_end_markers
from evidence_io import write_json_atomic


def prepare(case, output):
    if output.resolve().is_relative_to(case.resolve()):
        raise ValueError("Output must be separate from source")
    if output.exists() and any(output.iterdir()):
        raise ValueError("Use a new empty directory")
    tree = ET.parse(case / "world.sdf")
    original = copy.deepcopy(tree)
    for plug in ("P1", "P2", "P3"):
        terminal = tree.find(f"world/model[@name='wire_harness_multi']/link[@name='{plug}']")
        if terminal is None:
            raise ValueError("Known three-connector harness required")
        old = [v for v in terminal.findall("visual") if v.get("name", "").startswith("end_tag_")]
        if not old:
            raise ValueError("Existing end tags required for a controlled comparison")
        for visual in old:
            terminal.remove(visual)
        add_end_markers(terminal, plug)

    def without_tags(value):
        value = copy.deepcopy(value)
        for link in value.findall("world/model/link"):
            for visual in list(link.findall("visual")):
                if visual.get("name", "").startswith("end_tag_"):
                    link.remove(visual)
        for element in value.iter():
            element.tail = None
            if element.text is not None and not element.text.strip():
                element.text = None
        return ET.tostring(value.getroot())

    if without_tags(original) != without_tags(tree):
        raise ValueError("Unexpected change outside visual tags")
    shutil.copytree(case, output, dirs_exist_ok=True)
    ET.indent(tree)
    tree.write(output / "world.sdf", encoding="utf-8", xml_declaration=True)
    write_json_atomic(
        output / "flat_marker_model.json",
        {
            "source_world_sha256": hashlib.sha256((case / "world.sdf").read_bytes()).hexdigest(),
            "world_sha256": hashlib.sha256((output / "world.sdf").read_bytes()).hexdigest(),
            "tag_plane_offset_m": END_OFFSET_M,
            "all_elements_except_visual_tags_identical": True,
            "scope": "Coplanar visual marker correction only; collisions, inertia, robot control, initial placement, drawing and acceptance unchanged. No perception or physical success evidence",
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
    parser.add_argument("--case", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    prepare(args.case, args.output)
