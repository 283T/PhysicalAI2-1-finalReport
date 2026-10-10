import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_contacts import CONTACT_TOPIC, add_arm_contact_sensors, inter_arm_pair


class ArmContactsTests(unittest.TestCase):
    def test_pairs_are_scoped_and_ignore_objects_and_same_arm(self):
        self.assertTrue(
            inter_arm_pair(
                "left_arm::left_arm_elbow::collision", "right_arm::right_arm_elbow::collision"
            )
        )
        self.assertTrue(inter_arm_pair("right_arm_finger::collision", "left_arm_wrist::collision"))
        self.assertFalse(inter_arm_pair("left_arm::finger", "wire_harness_multi::P1"))
        self.assertFalse(inter_arm_pair("left_arm::elbow", "left_arm::wrist"))
        self.assertFalse(inter_arm_pair("other_left_arm::collision", "right_arm::collision"))

    def test_every_left_collision_gets_a_sensor(self):
        world = ET.fromstring(
            '<world><model name="left_arm"><link name="a">'
            '<collision name="c1"/><collision name="c2"/></link>'
            '<link name="b"><collision name="c3"/></link></model></world>'
        )
        add_arm_contact_sensors(world)
        sensors = world.findall("model/link/sensor")
        self.assertEqual(len(sensors), 3)
        self.assertEqual({s.findtext("contact/collision") for s in sensors}, {"c1", "c2", "c3"})
        self.assertTrue(all(s.findtext("topic") == CONTACT_TOPIC for s in sensors))
        self.assertTrue(all(s.findtext("contact/topic") == CONTACT_TOPIC for s in sensors))
        self.assertEqual(world.find("plugin").get("name"), "gz::sim::systems::Contact")


if __name__ == "__main__":
    unittest.main()
