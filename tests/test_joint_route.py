import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from joint_route import joint_route


class JointRouteTest(unittest.TestCase):
    def test_hard_stop_roundoff_is_normalized_but_real_limit_excess_is_rejected(self):
        tiny = joint_route([-2.8e-14], [0.5], [[0, 1]], lambda q: True)
        self.assertTrue(tiny["success"])
        self.assertEqual(tiny["path"][0], [0.0])
        self.assertEqual(tiny["start_roundoff_correction_rad"], [2.8e-14])
        outside = joint_route([-1e-6], [0.5], [[0, 1]], lambda q: True)
        self.assertFalse(outside["success"])
        self.assertIsNone(outside["path"])

    def test_routes_around_obstacle_and_checks_shortcuts(self):
        def valid(q):
            return not (0.4 <= q[0] <= 0.6 and -0.25 <= q[1] <= 0.25)

        result = joint_route([0, 0], [1, 0], [[-0.3, 1.3], [-0.8, 0.8]], valid)
        self.assertTrue(result["success"])
        path = np.asarray(result["path"])
        np.testing.assert_array_equal(path[0], [0, 0])
        np.testing.assert_array_equal(path[-1], [1, 0])
        for a, b in zip(path[:-1], path[1:]):
            self.assertTrue(all(valid(a + (b - a) * t) for t in np.linspace(0, 1, 501)))

    def test_unreachable_and_colliding_endpoints_never_return_a_route(self):
        limits = [[-1, 1], [-1, 1]]
        self.assertFalse(joint_route([0, 0], [0.5, 0], limits, lambda q: False)["success"])
        result = joint_route([-0.5, 0], [0.5, 0], limits, lambda q: abs(q[0]) > 0.1, max_nodes=40)
        self.assertFalse(result["success"])
        self.assertIsNone(result["path"])


if __name__ == "__main__":
    unittest.main()
