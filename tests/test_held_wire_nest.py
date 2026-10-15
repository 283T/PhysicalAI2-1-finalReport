import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from held_wire_nest import place_in_nest, seat_check


class HeldWireNestTest(unittest.TestCase):
    def test_failed_placement_cannot_keep_the_pickup_success_flag(self):
        report = {"success": True}
        args = SimpleNamespace(output=Path("unused"), inspection_calibration=Path("unused"))
        with patch("held_wire_nest.acquire_observation", side_effect=RuntimeError("occluded")):
            with self.assertRaisesRegex(RuntimeError, "occluded"):
                place_in_nest(None, args, report, lambda phase: None, {})
        self.assertFalse(report["success"])
        self.assertTrue(report["dual_pickup_success"])

    def test_seat_requires_two_seconds_even_when_legacy_bench_accepts_one(self):
        goal = np.eye(4)
        for duration, expected in [(1.5, False), (2.1, True)]:
            rows = [
                {"time_s": t, "plug": [0, 0, 0], "plug_rotation_world": np.eye(3)}
                for t in np.linspace(0, duration, 20)
            ]
            with patch("held_wire_nest.evaluation_state", side_effect=lambda row, *args: row):
                self.assertEqual(seat_check(rows, goal)["success"], expected)


if __name__ == "__main__":
    unittest.main()
