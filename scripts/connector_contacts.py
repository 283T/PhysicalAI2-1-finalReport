"""Connector contact telemetry; intrinsic physics shapes are never changed."""

import argparse
import hashlib
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from arm_contacts import CONTACT_TOPIC
from build_world import child
from evidence_io import write_json_atomic


def validate_connector_sensors(world, plug):
    link = world.find(f"model[@name='wire_harness_multi']/link[@name='{plug}']")
    if plug not in ("P1", "P2", "P3") or link is None:
        raise ValueError("Known harness connector required")
    collisions = {c.get("name") for c in link.findall("collision")}
    sensors = [
        s for s in link.findall("sensor[@type='contact']") if s.findtext("topic") == CONTACT_TOPIC
    ]
    references = [s.findtext("contact/collision") for s in sensors]
    if len(collisions) != 3 or len(references) != 3 or set(references) != collisions:
        raise ValueError("One live contact sensor per housing/pin collision required")
    for sensor in sensors:
        rate = float(sensor.findtext("update_rate", "nan"))
        if (
            not math.isfinite(rate)
            or rate <= 0
            or sensor.findtext("always_on") != "true"
            or sensor.findtext("contact/topic") != CONTACT_TOPIC
        ):
            raise ValueError("Live, positive-rate connector contact telemetry required")
    return len(sensors)


def add_connector_sensors(world):
    if not any(p.get("name") == "gz::sim::systems::Contact" for p in world.findall("plugin")):
        raise ValueError("Existing Gazebo contact system required")
    model = world.find("model[@name='wire_harness_multi']")
    if model is None:
        raise ValueError("Known multi-connector harness required")
    for plug in ("P1", "P2", "P3"):
        link = model.find(f"link[@name='{plug}']")
        if link is None or link.findall("sensor[@type='contact']"):
            raise ValueError("New connector sensors required; preserve previous instrumentation")
        collisions = link.findall("collision")
        if len(collisions) != 3:
            raise ValueError("Known housing and two pins required")
        for index, collision in enumerate(collisions):
            sensor = child(link, "sensor", name=f"connector_contact_{index}", type="contact")
            child(sensor, "always_on", "true")
            child(sensor, "update_rate", 30)
            child(sensor, "topic", CONTACT_TOPIC)
            contact = child(sensor, "contact")
            child(contact, "collision", collision.get("name"))
            child(contact, "topic", CONTACT_TOPIC)
        validate_connector_sensors(world, plug)


def prepare(case, output):
    if output.resolve().is_relative_to(case.resolve()) or output.exists() and any(output.iterdir()):
        raise ValueError("New separate case directory required")
    tree = ET.parse(case / "world.sdf")
    world = tree.getroot().find("world")
    add_connector_sensors(world)
    shutil.copytree(case, output, dirs_exist_ok=True)
    ET.indent(tree)
    tree.write(output / "world.sdf", encoding="utf-8", xml_declaration=True)
    write_json_atomic(
        output / "connector_contact_telemetry.json",
        {
            "source_world_sha256": hashlib.sha256((case / "world.sdf").read_bytes()).hexdigest(),
            "sensors_added": 9,
            "scope": "Contact telemetry only; robot, connector, wire and fixture physics unchanged. No insertion or contact success evidence",
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
