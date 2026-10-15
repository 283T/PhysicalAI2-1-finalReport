import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from arm_kinematics import ArmKinematics, axis_rotation, rotation_error, transform


class ArmKinematicsTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        path = Path(self.directory.name) / "planar.urdf"
        path.write_text("""<robot name="planar">
          <joint name="q1" type="revolute"><parent link="world"/><child link="a"/>
            <axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14"/></joint>
          <joint name="q2" type="revolute"><parent link="a"/><child link="b"/>
            <origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14"/></joint>
          <joint name="tool" type="fixed"><parent link="b"/><child link="tip"/>
            <origin xyz="1 0 0"/></joint></robot>""")
        self.arm = ArmKinematics(path, "tip", tip_offset=(0, 0, 0))

    def test_forward_matches_analytic_planar_result_and_jacobian(self):
        q = np.array([0.4, -0.7])
        pose, jacobian = self.arm.forward(q, jacobian=True)
        np.testing.assert_allclose(
            pose[:3, 3], [math.cos(q[0]) + math.cos(sum(q)), math.sin(q[0]) + math.sin(sum(q)), 0]
        )
        for i in range(2):
            perturbed = q.copy()
            perturbed[i] += 1e-6
            other = self.arm.forward(perturbed)
            derivative = np.r_[
                (other[:3, 3] - pose[:3, 3]) / 1e-6,
                rotation_error(other[:3, :3], pose[:3, :3]) / 1e-6,
            ]
            np.testing.assert_allclose(jacobian[:, i], derivative, atol=2e-6)

    def test_inverse_reaches_known_pose_without_exceeding_limits(self):
        target = self.arm.forward([1.1, -0.6])
        result = self.arm.inverse(target)
        self.assertTrue(result["success"])
        q = np.array(result["q_rad"])
        self.assertTrue(((q >= self.arm.limits[:, 0]) & (q <= self.arm.limits[:, 1])).all())
        np.testing.assert_allclose(self.arm.forward(q)[:3, 3], target[:3, 3], atol=0.001)

    def test_strict_inverse_does_not_accept_coarse_solution(self):
        target = self.arm.forward([1.1, -0.6])
        coarse = self.arm.inverse(target)
        strict = self.arm.inverse(
            target,
            seed=coarse["q_rad"],
            attempts=1,
            position_tolerance=0.0001,
            angle_tolerance=0.002,
        )
        self.assertTrue(strict["success"])
        self.assertLess(
            np.linalg.norm(self.arm.forward(strict["q_rad"])[:3, 3] - target[:3, 3]), 0.0001
        )
        self.assertLess(
            np.linalg.norm(
                rotation_error(self.arm.forward(strict["q_rad"])[:3, :3], target[:3, :3])
            ),
            0.002,
        )
        for value in (0, -1, float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.arm.inverse(target, position_tolerance=value)
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.arm.inverse(target, angle_tolerance=value)

    def test_unreachable_pose_is_reported_as_failure(self):
        result = self.arm.inverse(transform([3, 0, 0]), attempts=3)
        self.assertFalse(result["success"])
        self.assertGreater(result["position_error_m"], 0.9)

    def test_invalid_target_transform_is_rejected(self):
        for target in (np.zeros((4, 4)), np.full((4, 4), np.nan)):
            with self.assertRaises(ValueError):
                self.arm.inverse(target)
        reflected = np.eye(4)
        reflected[0, 0] = -1
        with self.assertRaises(ValueError):
            self.arm.inverse(reflected)

    def test_rotation_log_handles_exact_half_turn(self):
        target = axis_rotation([1, 0, 0], math.pi)
        error = rotation_error(target, np.eye(3))
        self.assertAlmostEqual(np.linalg.norm(error), math.pi)
        np.testing.assert_allclose(axis_rotation(error, np.linalg.norm(error)), target, atol=1e-8)
