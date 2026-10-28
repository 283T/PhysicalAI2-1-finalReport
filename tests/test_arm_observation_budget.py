import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_observation import acquire_observation


class ObservationWallBudgetTest(unittest.TestCase):
    def run_observer(self, session, elapsed, returncode=0):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            expected = {"connectors": [{"plug_id": "P3"}]}
            (output / "detections.json").write_text(json.dumps(expected))
            process = Mock(returncode=returncode)
            process.poll.side_effect = [None, returncode]
            with patch("arm_observation.subprocess.Popen", return_value=process) as start, \
                 patch("arm_observation.time.monotonic", side_effect=[0, elapsed]), \
                 patch("arm_observation.stop_process") as stop:
                result = acquire_observation(session, output)
                command = start.call_args.args[0]
                stop.assert_called_once_with(process, group=True)
            return result, command, expected

    def test_longer_wall_wait_keeps_commands_alive_and_requires_successful_observer(self):
        session = SimpleNamespace(observation_timeout_wall_s=240, tick=Mock())
        result, command, expected = self.run_observer(session, 130)
        self.assertEqual(result, expected)
        self.assertIn("--require-6d", command)
        self.assertEqual(command[command.index("--timeout") + 1], "240.0")
        session.tick.assert_called_once()
        with self.assertRaisesRegex(RuntimeError, "Fresh observation rejected"):
            self.run_observer(session, 130, returncode=1)

    def test_default_parent_wall_limit_remains_120_seconds(self):
        session = SimpleNamespace(tick=Mock())
        with self.assertRaisesRegex(RuntimeError, "Fresh observation timed out"):
            self.run_observer(session, 130)
        session.tick.assert_not_called()

    def test_invalid_budget_cannot_launch_observer(self):
        for timeout in (float("nan"), float("inf"), 0, 301):
            with tempfile.TemporaryDirectory() as folder, \
                 patch("arm_observation.subprocess.Popen") as start:
                with self.assertRaises(ValueError):
                    acquire_observation(SimpleNamespace(observation_timeout_wall_s=timeout), Path(folder))
                start.assert_not_called()


if __name__ == "__main__":
    unittest.main()
