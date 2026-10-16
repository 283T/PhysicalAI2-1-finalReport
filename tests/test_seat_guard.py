import copy
import io
import json
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from seat_guard import end_nest_guard_for_pickup, register_seat_guard


class Session:
    def __init__(self):
        self.evaluation_guards = []
        self.row = {
            "sim_time_s": 0,
            "phase": "park",
            "state": {"plug": [0, 0, 0], "plug_rotation_world": np.eye(3).tolist()},
        }

    def tick(self):
        row = copy.deepcopy(self.row)
        for guard in list(self.evaluation_guards):
            guard(row)
        return row


class SeatGuardTests(unittest.TestCase):
    def setup_guard(self, kind="connected"):
        session = Session()
        report = {}
        stream = io.StringIO()
        guard = register_seat_guard(
            session, report, "P3", np.eye(4), lambda r: r["state"], stream, kind=kind
        )
        return session, report, stream, guard

    def test_disconnect_is_latched_even_if_object_returns_and_report_is_edited(self):
        session, report, stream, guard = self.setup_guard()
        session.row.update(sim_time_s=0.1)
        session.row["state"]["plug"][0] = 0.001
        with self.assertRaises(RuntimeError):
            session.tick()
        self.assertFalse(json.loads(stream.getvalue().splitlines()[-1])["check"]["success"])
        report["seat_guard_failures"] = {}
        session.row["state"]["plug"][0] = 0
        session.row["sim_time_s"] = 0.2
        with self.assertRaisesRegex(RuntimeError, "latched"):
            session.tick()
        self.assertTrue(guard.has_failed())

    def test_logging_metadata_cannot_reset_reference_or_loosen_live_limits(self):
        session, report, _, _ = self.setup_guard()
        report["seat_references"]["P3"]["reference"]["plug"][0] = 0.0011
        for key in report["seat_references"]["P3"]["criteria"]:
            if key.endswith("_exclusive"):
                report["seat_references"]["P3"]["criteria"][key] = 100
        session.row["sim_time_s"] = 0.1
        session.row["state"]["plug"][0] = 0.0011
        with self.assertRaises(RuntimeError):
            session.tick()

    def test_connected_monitor_cannot_be_removed_for_next_pickup(self):
        session, report, _, guard = self.setup_guard()
        report["seat_references"]["P3"]["kind"] = "passive_nest"
        with self.assertRaises(RuntimeError):
            end_nest_guard_for_pickup(session, report, "P3", guard, "new", lambda r: None)
        self.assertIn(guard, session.evaluation_guards)

    def test_passive_nest_to_grip_transition_requires_active_fresh_grip(self):
        session, report, _, guard = self.setup_guard("passive_nest")

        def grip(row):
            report["grip_guard_summaries"]["top_primary"].update(samples=1)

        with self.assertRaises(RuntimeError):
            end_nest_guard_for_pickup(session, report, "P3", guard, "top_primary", grip)
        report["grip_references"] = {"top_primary": {}}
        report["grip_guard_summaries"] = {"top_primary": {"samples": 0}}
        session.evaluation_guards.append(grip)
        session.row["sim_time_s"] = 0.1
        end_nest_guard_for_pickup(session, report, "P3", guard, "top_primary", grip)
        self.assertNotIn(guard, session.evaluation_guards)
        self.assertIn(grip, session.evaluation_guards)
        self.assertEqual(report["seat_guard_endings"]["P3"]["grip"], "top_primary")

    def test_passive_monitor_stays_active_while_waiting_for_next_grip_sample(self):
        session, report, _, guard = self.setup_guard("passive_nest")
        report["grip_references"] = {"top": {"time_s": 0.1}}
        report["grip_guard_summaries"] = {"top": {"samples": 0}}

        def grip(row):
            if row["sim_time_s"] > 0.1:
                report["grip_guard_summaries"]["top"]["samples"] += 1
            self.assertIn(guard, session.evaluation_guards)

        session.evaluation_guards.append(grip)
        tick = session.tick

        def next_tick():
            session.row["sim_time_s"] += 0.1
            return tick()

        session.tick = next_tick
        end_nest_guard_for_pickup(session, report, "P3", guard, "top", grip)
        self.assertAlmostEqual(report["seat_guard_endings"]["P3"]["sim_time_s"], 0.2)

    def test_nonfinite_observation_fails_and_evidence_remains_valid_json(self):
        session, report, stream, _ = self.setup_guard()
        session.row["sim_time_s"] = 0.1
        session.row["state"]["plug"][0] = float("nan")
        with self.assertRaises(RuntimeError):
            session.tick()
        self.assertIsNone(json.loads(stream.getvalue().splitlines()[-1])["observed"]["plug"][0])
        json.dumps(report, allow_nan=False)

    def test_evidence_write_error_cannot_erase_a_detected_disconnect(self):
        session, report, stream, guard = self.setup_guard()
        stream.close()
        session.row["sim_time_s"] = 0.1
        session.row["state"]["plug"][0] = 0.0011
        with self.assertRaises(ValueError):
            session.tick()
        self.assertTrue(guard.has_failed())
        self.assertIn("P3", report["seat_guard_failures"])
