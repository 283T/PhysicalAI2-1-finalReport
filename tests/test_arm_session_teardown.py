import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_session import ArmSession


class ArmSessionTeardownTest(unittest.TestCase):
    def test_carrier_threshold_failure_preserves_the_offending_sample(self):
        session = ArmSession.__new__(ArmSession)
        session.arms, session.joints = {}, {}
        session.phase = "lift"
        session.carrier_pose = np.eye(4)
        session.stop_diagnostics = None
        session.poses = (
            10,
            42,
            {
                "conveyor_carrier": {"xyz": [0.0006, 0, 0], "q": [0, 0, 0, 1]},
                "carrier": {"xyz": [0, 0, 0], "q": [0, 0, 0, 1]},
            },
        )
        with patch("arm_session.time.monotonic", return_value=10):
            with self.assertRaisesRegex(RuntimeError, "Carrier moved"):
                session.snapshot()
        self.assertEqual(session.stop_diagnostics["sim_time_s"], 42)
        self.assertEqual(session.stop_diagnostics["carrier_position_error_m"], 0.0006)
        self.assertEqual(session.stop_diagnostics["phase"], "lift")

    def test_invalid_geometry_snapshot_cannot_prevent_finger_open_or_cleanup(self):
        session = ArmSession.__new__(ArmSession)
        session.targets = {"left_arm": [0] * 6}
        session.poses = (0, 0, {})
        session.pubs = {"left_arm": [Mock() for _ in range(8)]}
        session.snapshot = Mock(side_effect=RuntimeError("Carrier moved"))
        session.node, session.transport, session.bridge = Mock(), Mock(), Mock()
        session.pose_topic = "/poses"
        with (
            patch("arm_session.time.monotonic", side_effect=[0, 0, 31]),
            patch("arm_session.rclpy.spin_once"),
            patch("arm_session.stop_process") as stop,
        ):
            with self.assertRaisesRegex(RuntimeError, "Final open sent"):
                session.close()
        session.snapshot.assert_not_called()
        for publisher in session.pubs["left_arm"][:6]:
            publisher.publish.assert_not_called()
        for publisher in session.pubs["left_arm"][-2:]:
            self.assertEqual(publisher.publish.call_args.args[0].data, 0.018)
        self.assertEqual(session.transport.unsubscribe.call_count, 2)
        session.node.destroy_node.assert_called_once()
        stop.assert_called_once()


if __name__ == "__main__":
    unittest.main()
