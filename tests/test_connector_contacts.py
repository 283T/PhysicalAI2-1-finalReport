import copy
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from connector_contacts import add_connector_sensors, validate_connector_sensors


def scene():
    world = ET.fromstring(
        '<world><plugin name="gz::sim::systems::Contact" filename="contact"/>'
        '<model name="wire_harness_multi"><link name="wire"><collision name="wire"/>'
        "</link></model></world>"
    )
    model = world.find("model")
    for plug in ("P1", "P2", "P3"):
        link = ET.SubElement(model, "link", name=plug)
        ET.SubElement(link, "inertial").text = "preserved"
        for name in ("housing", "pin0", "pin1"):
            ET.SubElement(link, "collision", name=name)
    return world


class ConnectorContactTests(unittest.TestCase):
    def test_only_nine_live_sensors_are_added(self):
        world = scene()
        original = ET.tostring(world)
        add_connector_sensors(world)
        self.assertEqual(len(world.findall(".//sensor")), 9)
        for plug in ("P1", "P2", "P3"):
            self.assertEqual(validate_connector_sensors(world, plug), 3)
        restored = copy.deepcopy(world)
        for link in restored.findall("model/link"):
            for sensor in link.findall("sensor"):
                link.remove(sensor)
        self.assertEqual(ET.tostring(restored), original)
        with self.assertRaises(ValueError):
            add_connector_sensors(world)

    def test_disabled_duplicate_or_mismatched_sensors_are_rejected(self):
        original = scene()
        add_connector_sensors(original)
        for path, text in (
            ("update_rate", "0"),
            ("update_rate", "nan"),
            ("always_on", "false"),
            ("contact/collision", "missing"),
            ("contact/topic", "/other"),
        ):
            world = copy.deepcopy(original)
            world.find("model/link[@name='P3']/sensor/" + path).text = text
            with self.assertRaises(ValueError):
                validate_connector_sensors(world, "P3")
        world = copy.deepcopy(original)
        link = world.find("model/link[@name='P3']")
        link.append(copy.deepcopy(link.find("sensor")))
        with self.assertRaises(ValueError):
            validate_connector_sensors(world, "P3")
