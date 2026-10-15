"""Build a separate, permanently stepped left-finger scene before a new trial."""

import argparse
import copy
import hashlib
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

from arm_contacts import CONTACT_TOPIC
from build_world import child
from evidence_io import write_json_atomic
from stepped_wire_profile import stepped_profile


def prepare(case, models, output):
    if any(output.resolve().is_relative_to(p.resolve()) for p in (case, models)):
        raise ValueError("Output must be separate from inputs")
    if output.exists() and any(output.iterdir()):
        raise ValueError("Use a new empty directory")
    tree = ET.parse(case / "world.sdf")
    baseline = copy.deepcopy(tree)
    world = tree.getroot().find("world")
    model = world.find("model[@name='left_arm']")
    if model is None:
        raise ValueError("Known left arm required")
    output.mkdir(parents=True, exist_ok=True)
    shutil.copytree(case, output / "case")
    shutil.copytree(models, output / "models")
    source_urdf = models / "left_arm.urdf"
    candidate = stepped_profile(ET.parse(source_urdf).getroot(), "left_arm_")
    destination = output / "models/left_arm.urdf"
    ET.ElementTree(candidate).write(destination, encoding="utf-8", xml_declaration=True)
    converted = ET.fromstring(subprocess.check_output(["gz", "sdf", "-p", str(destination)]))
    changed = []
    for side in ("left", "right"):
        name = "left_arm_finger_" + side
        current = model.find(f"link[@name='{name}']")
        replacement = converted.find(f"model/link[@name='{name}']")
        if current is None or replacement is None or len(replacement.findall("collision")) != 4:
            raise ValueError("Exactly four converted pads per known finger required")
        # Preserve joint-relative placement, but take mass, geometry and visuals
        # from the same converted URDF used by IK and collision screening.
        replacement = copy.deepcopy(replacement)
        pose = replacement.find("pose")
        if pose is not None:
            replacement.remove(pose)
        replacement.insert(0, copy.deepcopy(current.find("pose")))
        for index, collision in enumerate(replacement.findall("collision")):
            sensor = child(replacement, "sensor", name=f"inter_arm_contact_{index}", type="contact")
            child(sensor, "always_on", "true")
            child(sensor, "update_rate", 30)
            child(sensor, "topic", CONTACT_TOPIC)
            contact = child(sensor, "contact")
            child(contact, "collision", collision.get("name"))
            child(contact, "topic", CONTACT_TOPIC)
        position = list(model).index(current)
        model.remove(current)
        model.insert(position, replacement)
        changed.append(name)

    def without_changed_links(tree):
        tree = copy.deepcopy(tree)
        arm = tree.find("world/model[@name='left_arm']")
        for name in changed:
            arm.remove(arm.find(f"link[@name='{name}']"))
        for element in tree.iter():
            element.tail = None
            if element.text is not None and not element.text.strip():
                element.text = None
        return ET.tostring(tree.getroot())

    if without_changed_links(tree) != without_changed_links(baseline):
        raise ValueError("Unexpected change outside the two finger links")
    contacts = tree.findall("world/model/link/sensor[@type='contact']")
    source_contacts = baseline.findall("world/model/link/sensor[@type='contact']")
    if len(contacts) != len(source_contacts) + 4:
        raise ValueError("Each new contact pad must have a contact sensor")
    for link in tree.findall("world/model/link"):
        collisions = {c.get("name") for c in link.findall("collision")}
        if any(
            s.findtext("contact/collision") not in collisions
            for s in link.findall("sensor[@type='contact']")
        ):
            raise ValueError("Contact sensor references missing collision geometry")
    ET.indent(tree)
    tree.write(output / "case/world.sdf", encoding="utf-8", xml_declaration=True)
    write_json_atomic(
        output / "case/stepped_profile.json",
        {
            "source_world_sha256": hashlib.sha256((case / "world.sdf").read_bytes()).hexdigest(),
            "source_urdf_sha256": hashlib.sha256(source_urdf.read_bytes()).hexdigest(),
            "candidate_urdf_sha256": hashlib.sha256(destination.read_bytes()).hexdigest(),
            "changed_links": changed,
            "all_other_world_elements_identical": True,
            "source_contact_sensors": len(source_contacts),
            "candidate_contact_sensors": len(contacts),
            "all_contact_references_resolve": True,
            "scope": "Permanent geometry, visual, inertial and contact sensor change only; no physical grasp, transport or insertion validation",
        },
    )
    # Replace the copied case manifest, which described the previous world.
    for directory in (output / "case", output / "models", output):
        write_json_atomic(
            directory / "manifest.json",
            {
                str(p.relative_to(directory)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sorted(directory.rglob("*"))
                if p.is_file() and p != directory / "manifest.json"
            },
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("case", "models", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    prepare(args.case, args.models, args.output)
