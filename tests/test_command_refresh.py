import sys
import unittest
from pathlib import Path
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from command_refresh import CommandRefresh


class CommandRefreshTests(unittest.TestCase):
    def test_changed_targets_are_immediate_and_unchanged_targets_recover_by_refresh(self):
        commands = CommandRefresh()
        send = Mock()
        self.assertTrue(commands.publish(("right", 0), 1, 0, send))
        self.assertFalse(commands.publish(("right", 0), 1, 0.05, send))
        self.assertTrue(commands.publish(("right", 0), 1.000000001, 0.06, send))
        self.assertTrue(commands.publish(("right", 0), 1.000000001, 0.17, send))
        self.assertEqual([c.args[0] for c in send.call_args_list], [1, 1.000000001, 1.000000001])
        self.assertEqual(commands.summary["unchanged_suppressed"], 1)

    def test_channels_do_not_starve_each_other(self):
        commands = CommandRefresh()
        a, b = Mock(), Mock()
        commands.publish(("left", 0), 0, 0, a)
        commands.publish(("right", 0), 0, 0, b)
        commands.publish(("left", 0), 1, 0.05, a)
        self.assertTrue(commands.publish(("right", 0), 0, 0.11, b))
        self.assertEqual(b.call_count, 2)

    def test_failed_send_is_retried_and_invalid_command_never_sent(self):
        commands = CommandRefresh()
        send = Mock(side_effect=[RuntimeError("lost"), None])
        with self.assertRaises(RuntimeError):
            commands.publish("jaw", 0.018, 0, send)
        self.assertTrue(commands.publish("jaw", 0.018, 0.001, send))
        with self.assertRaises(ValueError):
            commands.publish("jaw", float("nan"), 1, send)
        self.assertEqual(send.call_count, 2)
