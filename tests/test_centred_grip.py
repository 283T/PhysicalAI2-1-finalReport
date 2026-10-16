import hashlib
import sys
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from centred_grip import CentredGrip, activate, finger_commands, finger_pid


class CentredGripTests(unittest.TestCase):
    def test_measured_midpoint_is_preserved_with_finite_physical_targets(self):
        source = [0.0020963, 0.0019258]
        plan = CentredGrip(source, 10, 0.0015, 3000, 40)
        source[0] = 0
        for stamp in (10, 11, 85, 200, 10000):
            commands = plan.commands(stamp)
            self.assertAlmostEqual(commands[0] - commands[1], 0.0020963 - 0.0019258, places=14)
            self.assertTrue(all(0 < x <= 0.02 for x in commands))
        with self.assertRaises(FrozenInstanceError):
            plan.squeeze_m = 0
        with self.assertRaises(ValueError):
            plan.commands(9)
        with self.assertRaises(ValueError):
            plan.commands(float("nan"))

    def test_ideal_blocked_joint_pi_output_stays_bounded_instead_of_winding_up(self):
        plan = CentredGrip((0.002, 0.002), 0, 0.0015, 3000, 40)
        integral = 0
        dt = 0.01
        outputs = []
        for index in range(15001):
            error = 0.002 - plan.commands(index * dt)[0]
            integral += error * dt
            outputs.append(3000 * error + 40 * integral)
        self.assertLess(max(abs(x - 4.5) for x in outputs), 0.001)
        fixed_target_output = 3000 * 0.0015 + 40 * 0.0015 * 150
        self.assertGreater(fixed_target_output, 12)
        # This ideal scalar model is not a physical holding/force benchmark.

    def test_zero_integral_keeps_squeeze_and_invalid_feedback_is_rejected(self):
        plan = CentredGrip((0.007, 0.007), 0, 0.0008, 10000, 0)
        self.assertEqual(plan.commands(0), plan.commands(500))
        for fingers in ((0, 0.004), (float("nan"), 0.002), (0.021, 0.002)):
            with self.assertRaises(ValueError):
                CentredGrip(fingers, 0, 0.0008, 3000, 40)
        self.assertEqual(finger_commands(0.018, None), (0.018, 0.018))
        self.assertEqual(finger_commands((0.001, 0.003), 0), (0.001, 0.003))
        for value in ((0.001,), (-0.001, 0.003), (float("nan"), 0.003)):
            with self.assertRaises(ValueError):
                finger_commands(value, 0)

    def test_world_pid_parameters_and_activation_preserve_original_grip_reference(self):
        root = Path(__file__).resolve().parents[1]
        world = (
            root
            / "docs/evaluation/cooperative_nest_20261016/preshape_nest_seat_wall_limit/case/world.sdf"
        )
        gains = finger_pid(world, "right_arm")
        self.assertEqual(gains["p_gain"], 3000)
        guard = object()
        session = SimpleNamespace(
            grips={"right_arm": 0},
            evaluation_guards=[guard],
            snapshot=lambda: {"sim_time_s": 10, "arms": {"right_arm": {"fingers": [0.002, 0.002]}}},
            dwell=Mock(),
        )
        reference = {"original_closure": True}
        report = {
            "world_sha256": hashlib.sha256(world.read_bytes()).hexdigest(),
            "grip_references": {"primary": reference},
        }
        args = SimpleNamespace(centred_grips=True, world=world)
        activate(
            session,
            args,
            report,
            Mock(),
            "right_arm",
            "primary",
            guard,
            0.0015,
            "primary_centred_hold",
        )
        self.assertIs(report["grip_references"]["primary"], reference)
        self.assertIsInstance(session.grips["right_arm"], CentredGrip)
        with self.assertRaises(RuntimeError):
            activate(session, args, report, Mock(), "right_arm", "primary", guard, 0.0015, "again")
        session.grips["right_arm"] = 0
        report.pop("centred_grip_plans")
        report["primary_guard_failure"] = {}
        with self.assertRaises(RuntimeError):
            activate(session, args, report, Mock(), "right_arm", "primary", guard, 0.0015, "failed")
