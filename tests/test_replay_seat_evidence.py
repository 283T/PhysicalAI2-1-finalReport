import copy
import io
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from replay_seat_evidence import replay
from seat_guard import register_seat_guard


class ReplaySeatEvidenceTests(unittest.TestCase):
    def fixture(self, *, fail=False):
        stream, report = io.StringIO(), {"success": not fail}

        def row(stamp, error=0):
            return {
                "phase": "socket_after_release" if stamp else "socket_open",
                "sim_time_s": stamp,
                "state": {
                    "time_s": stamp,
                    "plug": [0, 0, 0.23 + error],
                    "plug_rotation_world": np.eye(3).tolist(),
                },
            }

        session = SimpleNamespace(evaluation_guards=[], tick=lambda: row(0))
        goal = np.eye(4)
        goal[2, 3] = 0.23
        guard = register_seat_guard(
            session, report, "P3_connected", goal, lambda r: r["state"], stream, kind="connected"
        )
        for i in range(1, 27):
            try:
                guard(row(i * 0.1, 0.0011 if fail and i == 10 else 0))
            except RuntimeError:
                break
        return report, [json.loads(line) for line in stream.getvalue().splitlines()]

    def test_runtime_records_recompute_with_frozen_window_and_summary(self):
        report, records = self.fixture()
        result = replay(report, records)
        self.assertTrue(result["evidence_consistent"])
        self.assertTrue(result["all_recorded_seats_within_limits"])
        self.assertTrue(result["all_requested_windows_within_limits"])
        self.assertGreaterEqual(result["windows"]["P3_connected"]["duration_s"], 2)

    def test_failure_is_preserved_even_if_trial_success_flag_is_changed(self):
        report, records = self.fixture(fail=True)
        report["success"] = True
        result = replay(report, records)
        self.assertTrue(result["evidence_consistent"])
        self.assertFalse(result["all_recorded_seats_within_limits"])
        self.assertFalse(result["all_requested_windows_within_limits"])
        report.pop("seat_guard_failures")
        self.assertFalse(replay(report, records)["evidence_consistent"])

    def test_removed_or_modified_samples_and_thresholds_are_detected(self):
        report, records = self.fixture()
        for change in ("drop", "rewrite", "threshold", "ending"):
            r, rows = copy.deepcopy(report), copy.deepcopy(records)
            if change == "drop":
                rows.pop()
            elif change == "rewrite":
                rows[8]["check"]["success"] = False
            elif change == "threshold":
                r["seat_references"]["P3_connected"]["criteria"]["maximum_movement_m_exclusive"] = 1
            else:
                r["seat_guard_endings"] = {"P3_connected": {"sim_time_s": 2.6}}
            with self.subTest(change=change):
                self.assertFalse(replay(r, rows)["evidence_consistent"])

    def test_missing_expected_goal_or_window_never_looks_complete(self):
        report, records = self.fixture()
        result = replay(report, records, required_windows={"P1_connected": "final_after_release"})
        self.assertFalse(result["evidence_consistent"])
        self.assertFalse(result["all_requested_windows_within_limits"])
        expected = {
            name: {"kind": r["kind"], "goal_pose": r["goal_pose"]}
            for name, r in report["seat_references"].items()
        }
        expected["P3_connected"]["goal_pose"] = np.eye(4).tolist()
        self.assertFalse(replay(report, records, expected_seats=expected)["evidence_consistent"])

    def test_passive_seat_can_end_only_at_last_sample_with_active_new_grip(self):
        report, records = self.fixture()
        report["seat_references"]["P3_nest"] = report["seat_references"].pop("P3_connected")
        report["seat_references"]["P3_nest"]["kind"] = "passive_nest"
        report["seat_guard_summaries"]["P3_nest"] = report["seat_guard_summaries"].pop(
            "P3_connected"
        )
        for row in records:
            row["seat"] = "P3_nest"
        report["grip_references"] = {"P3_top_primary": {"time_s": 2.5}}
        report["grip_guard_summaries"] = {"P3_top_primary": {"samples": 1}}
        report["seat_guard_endings"] = {
            "P3_nest": {
                "sim_time_s": 2.6,
                "grip": "P3_top_primary",
                "reason": "intentional physical pickup with active grip monitoring",
            }
        }
        self.assertTrue(replay(report, records)["evidence_consistent"])
        report["grip_references"] = {}
        self.assertFalse(replay(report, records)["evidence_consistent"])
