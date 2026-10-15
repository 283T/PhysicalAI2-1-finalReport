import io
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from top_pickup_live import top_pickup, vision_body


def detection(plug="P3"):
    return {
        "connectors": [
            {
                "plug_id": plug,
                "pose_6d": {
                    "rotation_world_from_connector": np.eye(3).tolist(),
                    "center_xyz_m": [0, 0, 0.23],
                },
            }
        ]
    }


class TopPickupLiveTests(unittest.TestCase):
    def test_vision_reader_requires_exact_connector_and_rigid_pose(self):
        body = vision_body(detection())
        self.assertAlmostEqual(body[2, 3], 0.23)
        with self.assertRaises(ValueError):
            vision_body(detection("P2"))
        bad = detection()
        bad["connectors"][0]["pose_6d"]["rotation_world_from_connector"][0][0] = 2
        with self.assertRaises(ValueError):
            vision_body(bad)

    def test_no_actions_without_successful_physical_aftercare(self):
        session = Mock()
        with self.assertRaises(RuntimeError):
            top_pickup(session, SimpleNamespace(), {}, Mock(), object(), io.StringIO())
        self.assertEqual(session.mock_calls, [])

    def test_image_alignment_failure_never_closes_or_ends_nest_guard(self):
        row = {
            "arms": {
                "left_arm": {
                    "q_rad": [0] * 6,
                    "tcp": [0, 0, 0],
                    "tcp_rotation_world": np.eye(3).tolist(),
                }
            }
        }
        session = Mock()
        session.snapshot.return_value = row
        session.targets = {"left_arm": np.zeros(6)}
        session.arms = {"left_arm": Mock()}
        session.arms["left_arm"].forward.return_value = np.eye(4)
        session.grips = {"left_arm": 0.018}
        report = {"nest_aftercare_complete": True, "success": True}
        args = SimpleNamespace(output=Path("unused"), nest_calibration=Path("unused.json"))
        plan = {
            "connector_from_tool": np.eye(4).tolist(),
            "paths": {phase: {"left_arm": [[0] * 6, [0] * 6]} for phase in ("hover", "approach")},
        }
        with (
            patch("top_pickup_live.acquire_observation", return_value=detection()),
            patch("top_pickup_live.live_top_plan", return_value=plan),
            patch(
                "top_pickup_live.assess_preclose_pose",
                return_value={"success": False, "position_error_m": 0.003, "angle_error_rad": 0.03},
            ),
            patch("top_pickup_live.tracking_compensated_goal", return_value=np.eye(4)),
            patch("top_pickup_live.correction_path", return_value=np.zeros((2, 6))),
            patch("top_pickup_live.end_nest_guard_for_pickup") as end,
        ):
            with self.assertRaisesRegex(RuntimeError, "preclose rejected"):
                top_pickup(session, args, report, Mock(), object(), io.StringIO())
        self.assertFalse(report["success"])
        self.assertEqual(len(report["top_preclose_checks"]), 3)
        self.assertEqual(session.grips["left_arm"], 0.018)
        end.assert_not_called()
        session.dwell.assert_not_called()
