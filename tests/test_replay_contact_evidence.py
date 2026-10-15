import copy
import io
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_contacts import accumulate_contact
from carried_contact_guard import CarriedContactGuard
from replay_contact_evidence import payload_arm_audit, replay

PLUG = "wire_harness_multi::P3::housing_collision"
TARGET = "fixtures::board::socket_J3_0_0_collision"


class ReplayContactEvidenceTests(unittest.TestCase):
    def fixture(self, *, fail=False):
        report, stream, native = {"success": not fail}, io.StringIO(), {}
        guard = CarriedContactGuard("P3", "J3", 10, report, stream)
        guard.begin_descent(12)
        fixture = "conveyor_carrier::carrier::pallet" if fail else TARGET
        guard.observe(PLUG, fixture, 12.1, "socket_insert")
        accumulate_contact(native, "socket_insert", PLUG, fixture, [0.0001], 12.1)
        report["contacts"] = list(native.values())
        return report, [json.loads(line) for line in stream.getvalue().splitlines()]

    def test_runtime_native_contact_checks_recompute_without_force_claim(self):
        report, records = self.fixture()
        result = replay(report, records, {"P3": "J3"})
        self.assertTrue(result["evidence_consistent"])
        self.assertTrue(result["all_requested_target_contacts_recorded"])
        self.assertTrue(result["arm_contact_audit_passed"])

    def test_failure_remains_failed_if_controller_success_is_rewritten(self):
        report, records = self.fixture(fail=True)
        report["success"] = True
        result = replay(report, records, {"P3": "J3"})
        self.assertTrue(result["evidence_consistent"])
        self.assertFalse(result["all_requested_target_contacts_recorded"])
        self.assertEqual(len(result["recomputed_failures"]), 1)
        report.pop("carried_contact_failures")
        self.assertFalse(replay(report, records, {"P3": "J3"})["evidence_consistent"])

    def test_wrong_drawing_socket_removed_native_records_and_check_tampering_rejected(self):
        report, records = self.fixture()
        self.assertFalse(replay(report, records, {"P3": "J2"})["evidence_consistent"])
        for corruption in ("delete_native", "delete_processed", "rewrite_check"):
            r, rows = copy.deepcopy(report), copy.deepcopy(records)
            if corruption == "delete_native":
                r["contacts"] = []
            elif corruption == "delete_processed":
                rows = []
            else:
                rows[0]["check"]["success"] = False
            with self.subTest(corruption=corruption):
                self.assertFalse(replay(r, rows, {"P3": "J3"})["evidence_consistent"])

    def test_payload_touching_wrist_or_wrong_arm_is_not_hidden_by_grip_success(self):
        report, records = self.fixture()
        for link in (
            "left_arm::left_arm_wrist::collision",
            "right_arm::right_arm_finger_left::pad_collision",
        ):
            r = copy.deepcopy(report)
            r["contacts"].append(
                {"phase": "socket_insert", "collision1": PLUG, "collision2": link, "messages": 1}
            )
            result = replay(r, records, {"P3": "J3"})
            self.assertTrue(result["evidence_consistent"])
            self.assertFalse(result["arm_contact_audit_passed"])
        report["contacts"].append(
            {
                "phase": "socket_insert",
                "collision1": PLUG,
                "collision2": "left_arm::left_arm_finger_left::pad_collision",
                "messages": 1,
            }
        )
        self.assertTrue(replay(report, records, {"P3": "J3"})["arm_contact_audit_passed"])

    def test_recorded_inter_arm_collision_is_rejected_without_saved_failure_flag(self):
        report = {
            "contacts": [
                {
                    "phase": "socket_hover",
                    "collision1": "left_arm::wrist::c",
                    "collision2": "right_arm::wrist::c",
                    "messages": 1,
                }
            ]
        }
        self.assertEqual(len(payload_arm_audit(report)), 1)
