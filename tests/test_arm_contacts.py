import json
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_contacts import (
    CONTACT_TOPIC,
    InactivePayloadContactGuard,
    accumulate_contact,
    add_arm_contact_sensors,
    arm_fixture_pair,
    inactive_payload_arm_pair,
    inactive_payload_nest_pair,
    inter_arm_pair,
)


class ArmContactsTests(unittest.TestCase):
    def test_inactive_parts_may_rest_on_supply_but_cannot_occupy_active_nest(self):
        nest = "fixtures::fixture_link::regrasp_end_support_-1_collision"
        for part in ("P1", "P2", "P2_wire_3", "junction"):
            plug = f"wire_harness_multi::{part}::collision"
            for a, b in ((plug, nest), (nest, plug)):
                self.assertEqual(inactive_payload_nest_pair(a, b, {"P3"})["payload_link"], part)
            for allowed in (
                "conveyor_carrier::carrier::pallet",
                "fixtures::fixture_link::socket_J1_0_collision",
            ):
                self.assertIsNone(inactive_payload_nest_pair(plug, allowed, {"P3"}))
        self.assertIsNone(inactive_payload_nest_pair("wire_harness_multi::P3::pin", nest, {"P3"}))
        guard = InactivePayloadContactGuard({"P3"}, {})
        with self.assertRaisesRegex(RuntimeError, "non-active"):
            guard.observe("wire_harness_multi::P1::housing", nest, 10, "nest_transfer_4")

    def test_recorded_native_jam_stops_the_new_guard(self):
        path = (
            Path(__file__).resolve().parents[1]
            / "docs/evaluation/cooperative_nest_20261016/centred_unplanned_connector_contact/descent_all_contacts_probe2.json"
        )
        native = json.loads(path.read_text())
        report = {}
        guard = InactivePayloadContactGuard({"P3"}, report)
        with self.assertRaisesRegex(RuntimeError, "non-active"):
            for pair, observation in native["pairs"].items():
                first, second = pair.split(" || ")
                guard.observe(first, second, observation["first_sim_time_s"], "nest_transfer_4")
        self.assertEqual(report["inactive_payload_contact_failure"]["payload_link"], "P2")

    def test_nonactive_connectors_wire_and_shared_junction_cannot_touch_robot(self):
        for arm in ("left_arm", "right_arm"):
            robot = f"{arm}::{arm}_finger_right::pad"
            for link in ("P1", "P2", "P2_wire_3", "junction", "P30"):
                payload = f"wire_harness_multi::{link}::collision"
                for a, b in ((payload, robot), (robot, payload)):
                    with self.subTest(arm=arm, link=link, a=a):
                        self.assertEqual(
                            inactive_payload_arm_pair(a, b, {"P3"})["payload_link"], link
                        )
            for link in ("P3", "P3_wire_0", "P3_wire_8"):
                self.assertIsNone(
                    inactive_payload_arm_pair(f"wire_harness_multi::{link}::c", robot, {"P3"})
                )
        self.assertIsNone(
            inactive_payload_arm_pair(
                "wire_harness_multi::P2::c", "fixtures::fixture_link::c", {"P3"}
            )
        )
        self.assertIsNone(
            inactive_payload_arm_pair("other_harness::P2::c", "right_arm::finger::c", {"P3"})
        )

    def test_nonactive_contact_failure_latches_after_report_is_removed(self):
        report = {}
        guard = InactivePayloadContactGuard({"P3"}, report)
        finger = "right_arm::right_arm_finger_right::front_fence"
        guard.observe("wire_harness_multi::P3::housing", finger, 1, "primary_close")
        with self.assertRaisesRegex(RuntimeError, "non-active"):
            guard.observe("wire_harness_multi::P2::pin_1_collision", finger, 2, "nest_transfer_4")
        self.assertEqual(report["inactive_payload_contact_failure"]["payload_link"], "P2")
        report.pop("inactive_payload_contact_failure")
        with self.assertRaises(RuntimeError):
            guard.observe("wire_harness_multi::P3::housing", finger, 3, "nest_transfer_4")
        with self.assertRaises(ValueError):
            InactivePayloadContactGuard({"P3"}, report)

    def test_contact_episode_requires_explicit_intent_and_excludes_teardown(self):
        for plugs in (set(), {"P0"}, {"P3_wire_4"}):
            with self.assertRaises(ValueError):
                InactivePayloadContactGuard(plugs, {})
        report = {}
        guard = InactivePayloadContactGuard({"P3"}, report)
        guard.observe("wire_harness_multi::P2::c", "left_arm::finger::c", 10, "final_open")
        guard.check()
        self.assertNotIn("inactive_payload_contact_failure", report)

    def test_fixture_collision_gate_distinguishes_payload_telemetry(self):
        for fixture in ("fixtures::board::collision", "conveyor_carrier::carrier::pallet"):
            for arm in ("left_arm::finger::collision", "right_arm_wrist::collision"):
                self.assertTrue(arm_fixture_pair(arm, fixture))
                self.assertTrue(arm_fixture_pair(fixture, arm))
            self.assertFalse(arm_fixture_pair("wire_harness_multi::P3::housing", fixture))
            self.assertFalse(arm_fixture_pair("other_left_arm::collision", fixture))

    def test_contacts_are_grouped_by_phase_and_unordered_pair(self):
        records = {}
        accumulate_contact(records, "approach", "finger", "tray", [0.001], 1.0)
        accumulate_contact(records, "approach", "tray", "finger", [0.002, 0.0005], 1.2)
        accumulate_contact(records, "lift", "finger", "tray", [], 2.0)
        self.assertEqual(len(records), 2)
        row = records[("approach", "finger", "tray")]
        self.assertEqual(row["messages"], 2)
        self.assertEqual(row["max_depth_m"], 0.002)
        self.assertEqual(row["first_sim_time_s"], 1.0)
        self.assertEqual(row["last_sim_time_s"], 1.2)
        self.assertEqual(records[("lift", "finger", "tray")]["max_depth_m"], 0)

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

    def test_right_only_object_contacts_can_be_monitored(self):
        world = ET.fromstring(
            '<world><model name="left_arm"><link name="finger">'
            '<collision name="left"/></link></model>'
            '<model name="right_arm"><link name="finger">'
            '<collision name="right"/></link></model></world>'
        )
        add_arm_contact_sensors(world)
        for name, collision in (("left_arm", "left"), ("right_arm", "right")):
            sensors = world.findall(f"model[@name='{name}']/link/sensor")
            self.assertEqual(len(sensors), 1)
            self.assertEqual(sensors[0].findtext("contact/collision"), collision)
            self.assertEqual(sensors[0].findtext("topic"), CONTACT_TOPIC)


if __name__ == "__main__":
    unittest.main()
