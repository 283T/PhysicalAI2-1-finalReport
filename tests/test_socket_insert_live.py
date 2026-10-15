import io
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from socket_insert_live import insert_after_top_pickup


class SocketInsertLiveTests(unittest.TestCase):
    def test_no_insertion_actions_without_actual_top_lift_and_hold(self):
        for report in ({}, {"top_pickup_complete": True, "top_hold": {"success": False}}):
            session = Mock()
            with self.assertRaises(RuntimeError):
                insert_after_top_pickup(
                    session,
                    SimpleNamespace(),
                    report,
                    Mock(),
                    None,
                    object(),
                    io.StringIO(),
                    io.StringIO(),
                )
            self.assertEqual(session.mock_calls, [])

    def test_no_motion_or_opening_after_held_vision_failure(self):
        root = Path(__file__).resolve().parents[1]
        case = root / "docs/evaluation/cooperative_nest_20261016/hd_hover_wall_limit/case"
        plan = (
            root
            / "docs/evaluation/dual_pickup_20261016/support140_settle30_dual_hold/pickup/connection_plan.json"
        )
        args = SimpleNamespace(
            world=case / "world.sdf",
            drawing_plan=plan,
            output=Path("unused"),
            nest_calibration=Path("unused.json"),
        )
        session = Mock()
        session.grips = {"left_arm": 0}
        report = {"top_pickup_complete": True, "top_hold": {"success": True}, "success": True}
        with (
            patch("socket_insert_live.register_carried_contact_guard"),
            patch(
                "socket_insert_live.acquire_observation", side_effect=RuntimeError("image rejected")
            ),
            patch("socket_insert_live.end_grip_for_release") as end,
        ):
            with self.assertRaisesRegex(RuntimeError, "image rejected"):
                insert_after_top_pickup(
                    session, args, report, Mock(), None, object(), io.StringIO(), io.StringIO()
                )
        self.assertFalse(report["success"])
        self.assertNotIn("single_connector_complete", report)
        self.assertEqual(session.grips["left_arm"], 0)
        session.move.assert_not_called()
        end.assert_not_called()
