import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from nest_aftercare import live_parking_plan, release_support_and_park


class NestAftercareTests(unittest.TestCase):
    def test_unproven_nest_cannot_start_release_or_parking(self):
        session = Mock()
        with self.assertRaises(RuntimeError):
            release_support_and_park(session, None, {}, None, {}, None)
        session.tick.assert_not_called()

    def test_waiting_planner_keeps_monitoring_and_is_stopped_on_budget_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            args = SimpleNamespace(
                output=Path(directory), world=Path("world.sdf"), models=Path("models")
            )
            session = Mock()
            session.tick.return_value = {"sim_time_s": 1, "arms": {}}
            process = Mock()
            process.poll.return_value = None
            with (
                patch("nest_aftercare.subprocess.Popen", return_value=process),
                patch("nest_aftercare.time.monotonic", side_effect=[0, 0, 190]),
                patch("nest_aftercare.stop_process") as stop,
            ):
                with self.assertRaisesRegex(RuntimeError, "compute budget"):
                    live_parking_plan(session, args, {}, "left_arm")
                self.assertGreaterEqual(session.tick.call_count, 2)
                stop.assert_called_once_with(process, group=True)

    def test_seat_failure_while_planning_cannot_leave_planner_running(self):
        with tempfile.TemporaryDirectory() as directory:
            args = SimpleNamespace(
                output=Path(directory), world=Path("world.sdf"), models=Path("models")
            )
            session = Mock()
            session.tick.side_effect = [{"sim_time_s": 1, "arms": {}}, RuntimeError("seat shifted")]
            process = Mock()
            process.poll.return_value = None
            with (
                patch("nest_aftercare.subprocess.Popen", return_value=process),
                patch("nest_aftercare.time.monotonic", return_value=0),
                patch("nest_aftercare.stop_process") as stop,
            ):
                with self.assertRaisesRegex(RuntimeError, "seat shifted"):
                    live_parking_plan(session, args, {}, "left_arm")
                stop.assert_called_once_with(process, group=True)
