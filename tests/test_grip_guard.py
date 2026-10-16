import copy
import io
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from grip_guard import end_grip_for_release, register_grip_guard


class GripGuardTest(unittest.TestCase):
    def test_closure_reference_itself_is_not_counted_as_a_fresh_grip_sample(self):
        identity = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        reference = {
            "time_s": 1,
            "plug": [0, 0, 0],
            "tcp": [0, 0, 0],
            "plug_rotation_world": identity,
            "tcp_rotation_world": identity,
        }
        stream, report, session = io.StringIO(), {}, SimpleNamespace(evaluation_guards=[])
        guard = register_grip_guard(
            session, report, "primary", reference, lambda r: r["state"], stream
        )
        guard({"sim_time_s": 1, "phase": "close", "state": reference})
        self.assertEqual(report["grip_guard_summaries"]["primary"]["samples"], 0)
        fresh = copy.deepcopy(reference)
        fresh["time_s"] = 1.01
        guard({"sim_time_s": 1.01, "phase": "hold", "state": fresh})
        self.assertEqual(report["grip_guard_summaries"]["primary"]["samples"], 1)
        from replay_grip_evidence import replay

        self.assertTrue(replay(report, [json.loads(stream.getvalue())])["evidence_consistent"])

    def test_duplicate_episode_cannot_overwrite_original_reference(self):
        report = {"grip_references": {"primary": {"original": True}}}
        with self.assertRaises(ValueError):
            register_grip_guard(
                SimpleNamespace(evaluation_guards=[]),
                report,
                "primary",
                {},
                lambda row: row,
                io.StringIO(),
            )
        self.assertEqual(report["grip_references"]["primary"], {"original": True})

    def test_intentional_release_ends_only_that_arm_without_erasing_evidence(self):
        primary, support = object(), object()
        session = SimpleNamespace(
            evaluation_guards=[primary, support],
            tick=lambda: {"sim_time_s": 12, "phase": "opening"},
        )
        report = {"grip_guard_summaries": {"primary": {"samples": 42}}}
        end_grip_for_release(session, report, "primary", primary)
        self.assertEqual(session.evaluation_guards, [support])
        self.assertEqual(report["grip_guard_summaries"]["primary"]["samples"], 42)
        self.assertEqual(report["grip_guard_endings"]["primary"]["sim_time_s"], 12)
        with self.assertRaisesRegex(RuntimeError, "inactive or failed"):
            end_grip_for_release(session, report, "primary", primary)

    def test_a_failed_grip_cannot_be_disabled_as_a_planned_release(self):
        guard = object()
        session = SimpleNamespace(evaluation_guards=[guard])
        with self.assertRaisesRegex(RuntimeError, "inactive or failed"):
            end_grip_for_release(session, {"primary_guard_failure": {}}, "primary", guard)
        self.assertEqual(session.evaluation_guards, [guard])

    def test_small_per_phase_drift_cannot_reset_the_original_closure_reference(self):
        identity = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        reference = {
            "plug": [0, 0, 0],
            "tcp": [0, 0, 0],
            "plug_rotation_world": identity,
            "tcp_rotation_world": identity,
        }
        stream, report, session = io.StringIO(), {}, SimpleNamespace(evaluation_guards=[])
        guard = register_grip_guard(
            session, report, "primary", reference, lambda row: row["state"], stream
        )
        for stamp, drift in [(1, 0.003), (2, 0.006)]:
            state = copy.deepcopy(reference)
            state["plug"][0] = drift
            row = {"phase": "phase" + str(stamp), "sim_time_s": stamp, "state": state}
            if stamp == 1:
                guard(row)
            else:
                with self.assertRaisesRegex(RuntimeError, "closure reference"):
                    guard(row)
        self.assertEqual(report["grip_guard_summaries"]["primary"]["samples"], 2)
        self.assertEqual(report["primary_guard_failure"]["position_shift_m"], 0.006)
        self.assertEqual(len([json.loads(line) for line in stream.getvalue().splitlines()]), 2)


if __name__ == "__main__":
    unittest.main()
