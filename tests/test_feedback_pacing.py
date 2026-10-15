import copy
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_kinematics import rotation_error, transform
from feedback_pacing import FeedbackPacer


class Arm:
    def forward(self, q):
        return transform(q[:3], [0, 0, q[3]])


class FeedbackPacingTest(unittest.TestCase):
    def setup_motion(self):
        arms = {"a": Arm(), "b": Arm()}
        paths = {
            "a": np.array([[0, 0, 0.3, 0], [0.1, 0, 0.3, 0.3]]),
            "b": np.array([[0, 0.2, 0.3, 0], [0.1, 0.2, 0.3, 0.3]]),
        }
        return arms, FeedbackPacer(arms, paths, 10, 0)

    def feedback(self, arms, commands, stamp):
        return {
            "sim_time_s": stamp,
            "arms": {
                n: {
                    "q_rad": commands[n].tolist(),
                    "tcp": a.forward(commands[n])[:3, 3].tolist(),
                    "tcp_rotation_world": a.forward(commands[n])[:3, :3].tolist(),
                }
                for n, a in arms.items()
            },
        }

    def test_one_lagging_arm_stops_both_and_wait_cannot_create_a_command_jump(self):
        arms, pacer = self.setup_motion()
        initial = pacer.commands(0)
        command = pacer.update(self.feedback(arms, initial, 0.1))
        fraction = pacer.fraction
        for stamp in (0.2, 100):
            row = self.feedback(arms, command, stamp)
            row["arms"]["b"]["tcp"][0] -= 0.01
            held = pacer.update(row)
            self.assertEqual(pacer.fraction, fraction)
            for n in command:
                np.testing.assert_array_equal(held[n], command[n])
        advanced = pacer.update(self.feedback(arms, command, 100.1))
        self.assertGreater(pacer.fraction, fraction)
        for n in command:
            old, new = arms[n].forward(command[n]), arms[n].forward(advanced[n])
            self.assertLessEqual(np.linalg.norm(new[:3, 3] - old[:3, 3]), 0.0005)
            self.assertLessEqual(np.linalg.norm(rotation_error(new[:3, :3], old[:3, :3])), 0.003)
        self.assertEqual(pacer.summary["paused_samples"], 2)

    def test_duplicate_observation_and_invalid_feedback_cannot_advance(self):
        arms, pacer = self.setup_motion()
        commands = pacer.commands(0)
        row = self.feedback(arms, commands, 0)
        pacer.update(row)
        self.assertEqual(pacer.fraction, 0)
        invalid = copy.deepcopy(row)
        invalid["sim_time_s"] = 1
        invalid["arms"]["a"]["tcp"][0] = float("nan")
        with self.assertRaisesRegex(RuntimeError, "Invalid robot tool feedback"):
            pacer.update(invalid)
        self.assertEqual(pacer.fraction, 0)

    def test_joint_lag_also_pauses_progress_even_if_tools_appear_aligned(self):
        arms, pacer = self.setup_motion()
        row = self.feedback(arms, pacer.commands(0), 1)
        row["arms"]["a"]["q_rad"][3] += 0.02
        pacer.update(row)
        self.assertEqual(pacer.fraction, 0)
        self.assertEqual(pacer.summary["paused_samples"], 1)


if __name__ == "__main__":
    unittest.main()
