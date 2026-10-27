import gzip
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from analyze_feedback_gates import analyze
from arm_kinematics import transform


class FeedbackGateAuditTest(unittest.TestCase):
    def fixture(self, root, order):
        gates = {"position_gate_m": 0.001, "angle_gate_rad": 0.01,
                 "joint_gate_rad": 0.01, "relative_position_gate_m": 0.0015,
                 "relative_angle_gate_rad": 0.015}
        report = {"trial_state": "finished", "success": False, "phase": "nest_transfer_0",
                  "preflights": [{"phase": "nest_transfer_0", "feedback_pacing": gates}]}
        plan = {"selected_attempt": 0, "attempts": [{"phases": [
            {"joint_paths": {n: [] for n in order}}]}]}
        states = {}
        for name, position, observed, yaw in [
            ("left_arm", [0, 0, 0], [0, 0, 0], 0.009),
            ("right_arm", [0.14, 0, 0], [0.14, -0.0003, 0], 0),
        ]:
            states[name] = {"tcp": observed,
                            "tcp_rotation_world": transform(observed, [0, 0, yaw])[:3, :3].tolist(),
                            "command_tcp_pose": transform(position, [0, 0, 0]).tolist(),
                            "q_rad": [yaw, 0], "command_q_rad": [0, 0]}
        rows = [{"phase": "nest_transfer_0", "sim_time_s": 1, "arms": states}]
        (root / "validation.json").write_text(json.dumps(report))
        (root / "nest_plan.json").write_text(json.dumps(plan))
        (root / "trajectory.json.gz").write_bytes(gzip.compress(json.dumps(rows).encode()))
        return report

    def test_uses_actual_path_order_for_orientation_dependent_relative_position_gate(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            self.fixture(root, ["left_arm", "right_arm"])
            forward = analyze(root)
            self.fixture(root, ["right_arm", "left_arm"])
            reverse = analyze(root)
        self.assertEqual(forward["relative"]["position_exceedances"], 1)
        self.assertEqual(reverse["relative"]["position_exceedances"], 0)
        self.assertFalse(forward["trial_success"])
        self.assertEqual(forward["recorded_pacer_summary"]["relative_position_gate_m"], 0.0015)
        self.assertEqual(forward["arms"]["left_arm"]["position_exceedances"], 0)
        self.assertEqual(forward["arms"]["right_arm"]["position_exceedances"], 0)
        self.assertTrue(np.isfinite(forward["relative"]["maximum_position_m"]))

    def test_live_trial_cannot_be_reclassified_by_offline_audit(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            report = self.fixture(root, ["left_arm", "right_arm"])
            report["trial_state"] = "running"
            (root / "validation.json").write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError, "Finished trial required"):
                analyze(root)


if __name__ == "__main__":
    unittest.main()
