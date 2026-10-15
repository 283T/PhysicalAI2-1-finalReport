import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from grasp_validation import assess_relative_grasp
from replay_grip_evidence import replay


class ReplayGripEvidenceTest(unittest.TestCase):
    def fixture(self):
        identity = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        reference = {
            "time_s": 1,
            "plug": [0, 0, 0],
            "tcp": [0, 0, 0],
            "plug_rotation_world": identity,
            "tcp_rotation_world": identity,
        }
        records = []
        for stamp, drift in [(2, 0.003), (3, 0.006)]:
            observed = copy.deepcopy(reference)
            observed.update(time_s=stamp, plug=[drift, 0, 0])
            records.append(
                {
                    "grip": "primary",
                    "phase": "move",
                    "sim_time_s": stamp,
                    "observed": observed,
                    "check": assess_relative_grasp(reference, observed),
                }
            )
        report = {
            "success": False,
            "grip_references": {"primary": reference},
            "grip_guard_summaries": {
                "primary": {
                    "samples": 2,
                    "maximum_position_shift_m": 0.006,
                    "maximum_angle_shift_rad": 0.0,
                }
            },
        }
        return report, records

    def test_consistent_failure_remains_a_failure(self):
        result = replay(*self.fixture())
        self.assertTrue(result["evidence_consistent"])
        self.assertFalse(result["all_recorded_grips_within_limits"])
        self.assertFalse(result["trial_success"])
        self.assertEqual(len(result["failed_observations"]), 1)

    def test_omitting_last_failure_is_detected_by_sample_count(self):
        report, records = self.fixture()
        self.assertFalse(replay(report, records[:-1])["evidence_consistent"])

    def test_overriding_recorded_success_does_not_override_recomputation(self):
        report, records = self.fixture()
        records[-1]["check"]["success"] = True
        result = replay(report, records)
        self.assertFalse(result["evidence_consistent"])
        self.assertEqual(len(result["failed_observations"]), 1)

    def test_release_event_cannot_hide_later_recorded_slip(self):
        report, records = self.fixture()
        report["grip_guard_endings"] = {
            "primary": {"sim_time_s": 2, "reason": "intentional physical opening"}
        }
        result = replay(report, records)
        self.assertFalse(result["evidence_consistent"])
        self.assertEqual(len(result["failed_observations"]), 1)


if __name__ == "__main__":
    unittest.main()
