import io
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from carried_contact_guard import CarriedContactGuard, connector_fixture_pair

P3 = "factory::wire_harness_multi::P3::housing_collision"
J3 = "factory::fixtures::board::socket_J3_0_0_collision"
J2 = "factory::fixtures::board::socket_J2_0_0_collision"


class CarriedContactTests(unittest.TestCase):
    def guard(self):
        self.report, self.stream = {}, io.StringIO()
        return CarriedContactGuard("P3", "J3", 10.0, self.report, self.stream)

    def test_native_target_contact_required_after_descent_boundary(self):
        guard = self.guard()
        guard.begin_descent(12)
        with self.assertRaises(RuntimeError):
            guard.require_target_contact()
        guard.observe(J3, P3, 12.1, "insert")
        guard.require_target_contact()
        self.assertEqual(
            self.report["carried_contact_guards"]["P3"]["summary"]["target_messages"], 1
        )

    def test_late_contact_cannot_gain_retroactive_permission(self):
        guard = self.guard()
        guard.begin_descent(12)
        guard.observe(P3, J3, 11.9, "insert")
        self.report["carried_contact_guards"]["P3"]["summary"]["failed"] = False
        with self.assertRaises(RuntimeError):
            guard.check()
        guard.observe(P3, J3, 12.1, "insert")
        with self.assertRaises(RuntimeError):
            guard.require_target_contact()

    def test_wrong_socket_and_pallet_are_rejected_even_during_insertion(self):
        for fixture in (J2, "conveyor_carrier::carrier::pallet"):
            guard = self.guard()
            guard.begin_descent(12)
            guard.observe(P3, fixture, 13, "insert")
            with self.assertRaises(RuntimeError):
                guard.check()

    def test_historical_messages_ignored_invalid_timestamp_latches_failure(self):
        guard = self.guard()
        guard.observe(P3, J2, 9, "lift")
        guard.check()
        guard.observe(P3, J3, float("nan"), "insert")
        with self.assertRaises(RuntimeError):
            guard.check()
        rows = [json.loads(line) for line in self.stream.getvalue().splitlines()]
        self.assertTrue(rows[0]["check"]["historical_before_lift"])
        self.assertIsNone(rows[1]["sim_time_s"])

    def test_exact_plug_link_scoping(self):
        for unrelated in (
            "wire_harness_multi::P30::housing",
            "other::P3::housing",
            "wire_harness_multi::wire_P3_0::collision",
            "left_arm::finger",
        ):
            self.assertIsNone(connector_fixture_pair(unrelated, J3, "P3"))
        self.assertEqual(connector_fixture_pair(P3, J3, "P3"), J3)

    def test_permission_boundary_cannot_be_reset(self):
        guard = self.guard()
        with self.assertRaises(ValueError):
            guard.begin_descent(9)
        guard.begin_descent(12)
        with self.assertRaises(ValueError):
            guard.begin_descent(10)
