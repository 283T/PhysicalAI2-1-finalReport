import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import dual_pickup


class DualStageFailureTests(unittest.TestCase):
    def test_later_exception_cannot_keep_success_from_an_earlier_stage(self):
        root = Path(__file__).resolve().parents[1]
        case = (
            root
            / "docs/evaluation/cooperative_nest_20261016/hd_wire_approach_fixture_rejected/case"
        )
        source = root / "docs/evaluation/dual_pickup_20261016/support140_settle30_dual_hold"
        session = SimpleNamespace(
            wait_ready=Mock(),
            close=Mock(),
            phase="after_pickup",
            command_refresh=SimpleNamespace(summary={}),
            contacts={},
            unintended=[],
            stop_diagnostics=None,
            preflights=[],
            history=[],
        )

        def pickup(session, args, report, *unused):
            report["success"] = True
            return {}

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "trial"
            argv = [
                "dual_pickup.py",
                "--world",
                str(case / "world.sdf"),
                "--models",
                str(source / "models"),
                "--drawing-plan",
                str(source / "pickup/connection_plan.json"),
                "--output",
                str(output),
                "--place-in-nest",
            ]
            with (
                patch.object(sys, "argv", argv),
                patch("dual_pickup.rclpy.init"),
                patch("dual_pickup.rclpy.shutdown"),
                patch("dual_pickup.ArmSession", return_value=session),
                patch("dual_pickup.run", side_effect=pickup),
                patch("dual_pickup.place_in_nest", side_effect=RuntimeError("later stage failed")),
                patch("builtins.print"),
            ):
                with self.assertRaises(SystemExit):
                    dual_pickup.main()
            report = json.loads((output / "validation.json").read_text())
            self.assertEqual(report["trial_state"], "finished")
            self.assertFalse(report["success"])
            self.assertEqual(report["error"], "later stage failed")
            self.assertFalse(report["final_acceptance_eligible"])
