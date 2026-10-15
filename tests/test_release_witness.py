import copy
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from release_witness import witness


class ReleaseWitnessTests(unittest.TestCase):
    def fixture(self):
        goal = np.eye(4)
        goal[2, 3] = 0.15

        def row(phase, stamp, z, travel):
            return {
                "phase": phase,
                "sim_time_s": stamp,
                "arms": {
                    "left_arm": {
                        "tcp": [0, 0, z],
                        "fingers": [travel, travel],
                        "command_tcp_pose": np.eye(4).tolist(),
                        "command_q_rad": [0] * 6,
                    }
                },
                "evaluation_poses": {
                    "wire_harness_multi": {"xyz": [0, 0, 0], "q": [0, 0, 0, 1]},
                    "P3": {"xyz": [0, 0, 0.15], "q": [0, 0, 0, 1]},
                },
            }

        rows = [
            row("socket_before_release", 0, 0.144, 0.007),
            row("socket_open", 1, 0.144, 0.018),
            row("socket_withdraw", 2, 0.144, 0.018),
            row("socket_withdraw", 6, 0.224, 0.018),
        ]
        rows += [row("socket_after_release", 7 + i * 0.1, 0.224, 0.018) for i in range(26)]
        return rows, goal

    def test_native_opening_withdrawal_and_frozen_seat_window(self):
        rows, goal = self.fixture()
        result = witness(rows, goal, rest_gap_m=0.006)
        self.assertTrue(result["success"])
        self.assertTrue(result["physical_opening_observed"])
        self.assertAlmostEqual(result["minimum_native_upward_withdrawal_m"], 0.08)
        self.assertGreaterEqual(result["seat_window"]["duration_s"], 2)

    def test_commands_cannot_replace_physical_opening_or_withdrawal(self):
        source, goal = self.fixture()
        for failed_motion in ("opening", "withdrawal"):
            rows = copy.deepcopy(source)
            for row in rows:
                arm = row["arms"]["left_arm"]
                arm["command_finger_travel_m"] = 0.018
                arm["command_tcp_pose"][2][3] = 0.224
                if row["phase"] != "socket_before_release":
                    if failed_motion == "opening":
                        arm["fingers"] = [0.007, 0.007]
                    else:
                        arm["tcp"][2] = 0.144
            with self.subTest(failed_motion=failed_motion):
                self.assertFalse(witness(rows, goal, rest_gap_m=0.006)["success"])

    def test_one_closed_finger_wrong_seat_or_short_window_never_passes(self):
        source, goal = self.fixture()
        for failure in ("one_finger", "seat", "short_window", "missing_withdrawal"):
            rows = copy.deepcopy(source)
            for row in rows:
                if row["phase"] == "socket_after_release":
                    if failure == "one_finger":
                        row["arms"]["left_arm"]["fingers"][0] = 0.007
                    elif failure == "seat":
                        row["evaluation_poses"]["P3"]["xyz"][2] += 0.0011
                    elif failure == "short_window":
                        row["sim_time_s"] = 7 + (row["sim_time_s"] - 7) / 4
            if failure == "missing_withdrawal":
                rows = [r for r in rows if r["phase"] != "socket_withdraw"]
            with self.subTest(failure=failure):
                self.assertFalse(witness(rows, goal, rest_gap_m=0.006)["success"])
