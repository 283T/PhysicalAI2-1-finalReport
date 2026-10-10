import copy
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from cooperative_pick import (
    assess_cooperation,
    assign_pair,
    check_tool_clearance,
    shared_lift,
    upright_waist_path,
)


class LinearArm:
    def __init__(self, origin):
        self.origin = np.asarray(origin)

    def forward(self, q):
        pose = np.eye(4)
        pose[:3, 3] = self.origin + q
        return pose

    def inverse(self, pose, seed=None):
        return {"success": True, "q_rad": pose[:3, 3] - self.origin}


class CooperativePickTests(unittest.TestCase):
    def plan(self):
        return {
            "connectors": [
                {
                    "plug_id": terminal,
                    "grasp_candidate": {},
                    "arms": {
                        name: {
                            "pickup_hover": {"success": True, "q_rad": [cost]},
                            "pickup_grasp": {"success": True},
                            "socket_hover": {"success": True},
                        }
                        for name, cost in zip(("left_arm", "right_arm"), costs)
                    },
                }
                for terminal, costs in (("P1", (1, 9)), ("P2", (1, 2)))
            ]
        }

    def test_assignment_uses_distinct_arms_even_when_both_prefer_left(self):
        result = assign_pair(self.plan(), "P1", "P2")
        self.assertEqual(result["left_arm"]["plug_id"], "P1")
        self.assertEqual(result["right_arm"]["plug_id"], "P2")

    def test_unreachable_pair_and_duplicate_rejected(self):
        plan = self.plan()
        for row in plan["connectors"]:
            row["arms"]["right_arm"]["pickup_grasp"]["success"] = False
        with self.assertRaises(ValueError):
            assign_pair(plan, "P1", "P2")
        with self.assertRaises(ValueError):
            assign_pair(self.plan(), "P1", "P1")
        plan = copy.deepcopy(self.plan())
        plan["connectors"][0]["grasp_candidate"] = None
        with self.assertRaises(ValueError):
            assign_pair(plan, "P1", "P2")

    def test_shared_lift_preserves_distance_at_every_interpolation(self):
        arms = {"left_arm": LinearArm([0, 0, 0]), "right_arm": LinearArm([0.4, 0.1, 0])}
        targets, paths = shared_lift(arms, {name: np.zeros(3) for name in arms})
        for i in range(len(paths["left_arm"])):
            a = arms["left_arm"].forward(paths["left_arm"][i])[:3, 3]
            b = arms["right_arm"].forward(paths["right_arm"][i])[:3, 3]
            np.testing.assert_allclose(b - a, [0.4, 0.1, 0])
        self.assertAlmostEqual(targets["right_arm"][2, 3], 0.1)

    def test_crossing_paths_rejected_despite_separated_endpoints(self):
        arms = {"left_arm": LinearArm([0, 0, 0]), "right_arm": LinearArm([0, 0, 0])}
        paths = {
            "left_arm": np.array([[0.0, 0, 0], [0.4, 0, 0]]),
            "right_arm": np.array([[0.4, 0, 0], [0.0, 0, 0]]),
        }
        with self.assertRaises(ValueError):
            check_tool_clearance(arms, paths)

    def test_cooperation_rejects_spacing_change(self):
        closed = {
            "arms": {
                "left_arm": {"plug": [0, 0, 0], "tcp": [0, 0, 0]},
                "right_arm": {"plug": [0.4, 0, 0], "tcp": [0.4, 0, 0]},
            }
        }
        sample = copy.deepcopy(closed)
        for state in sample["arms"].values():
            state["plug"][2] += 0.1
            state["tcp"][2] += 0.1
        self.assertTrue(assess_cooperation(closed, [sample] * 10)["success"])
        sample["arms"]["right_arm"]["plug"][0] += 0.006
        self.assertFalse(assess_cooperation(closed, [sample] * 10)["success"])

    def test_waist_rotation_does_not_lower_the_arm(self):
        goal = np.array([-3.1, 1.2, 1.0, 2.8, -0.7, 1.4])
        path = upright_waist_path(np.zeros(6), goal)
        self.assertAlmostEqual(path[1][0], goal[0])
        np.testing.assert_array_equal(path[1][1:], np.zeros(5))
        np.testing.assert_array_equal(path[-1], goal)
        with self.assertRaises(ValueError):
            upright_waist_path(np.array([0, 0.1, 0, 0, 0, 0]), goal)

    def test_invalid_lift_rejected(self):
        for height in (0, -1, float("nan")):
            with self.assertRaises(ValueError):
                shared_lift({}, {}, height)


if __name__ == "__main__":
    unittest.main()
