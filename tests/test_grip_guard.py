import copy
import io
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from grip_guard import register_grip_guard


class GripGuardTest(unittest.TestCase):
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
