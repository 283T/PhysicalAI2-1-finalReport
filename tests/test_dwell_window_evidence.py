import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_session import ArmSession


class DwellWindowEvidenceTest(unittest.TestCase):
    def session(self, fail_at=None):
        session = ArmSession.__new__(ArmSession)
        session.poses = (0, 10.0, {})
        session.snapshot = Mock(side_effect=lambda: {"sim_time_s": session.poses[1]})
        session.dwell_windows = []
        count = [0]

        def tick(**kwargs):
            count[0] += 1
            if count[0] == fail_at:
                raise RuntimeError("guard rejected contact")
            session.poses = (0, 10 + count[0] * 0.2, {})
            return {"sim_time_s": session.poses[1]}

        session.tick = tick
        return session

    def test_actual_start_end_and_unique_times_cover_the_unchanged_four_second_dwell(self):
        session = self.session()
        with patch("arm_session.time.monotonic", return_value=0):
            rows = session.dwell("dual_hold", 4)
        self.assertEqual(len(rows), 20)
        self.assertEqual(session.dwell_windows, [{
            "phase": "dual_hold", "start_sim_time_s": 10, "end_sim_time_s": 14,
            "requested_duration_s": 4, "elapsed_simulation_s": 4,
            "completed": True, "distinct_observation_timestamps": 20}])

    def test_guard_failure_keeps_partial_window_and_propagates_without_success(self):
        session = self.session(fail_at=3)
        with patch("arm_session.time.monotonic", return_value=0):
            with self.assertRaisesRegex(RuntimeError, "guard rejected"):
                session.dwell("dual_hold", 4)
        window = session.dwell_windows[0]
        self.assertFalse(window["completed"])
        self.assertEqual(window["distinct_observation_timestamps"], 2)
        self.assertAlmostEqual(window["elapsed_simulation_s"], 0.4)


if __name__ == "__main__":
    unittest.main()
