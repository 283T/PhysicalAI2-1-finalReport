import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from simulation_budget import SimulationBudget, dwell_wall_budget


class SimulationBudgetTests(unittest.TestCase):
    def test_loaded_hd_dwell_keeps_physical_duration_and_finite_compute_limits(self):
        budget = SimulationBudget(0, 0, 2.5, dwell_wall_budget(2.5))
        self.assertIsNone(budget.reason(2.402, 130.001))
        self.assertEqual(budget.reason(2.5, 136), "simulation_budget_exhausted")
        self.assertEqual(budget.summary["simulation_budget_s"], 2.5)
        slow = SimulationBudget(0, 0, 2.5, dwell_wall_budget(2.5))
        for wall in range(1, 360):
            self.assertIsNone(slow.reason(wall / 200, wall))
        self.assertEqual(slow.reason(1.8, 360), "wall_budget_exhausted")
        for duration in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                dwell_wall_budget(duration)

    def test_slow_progress_gets_compute_time_but_same_physical_duration_limit(self):
        budget = SimulationBudget(0, 0, 10, 400)
        for wall in range(1, 300):
            self.assertIsNone(budget.reason(wall / 30, wall))
        self.assertEqual(budget.reason(10, 300), "simulation_budget_exhausted")
        self.assertEqual(budget.summary["simulation_budget_s"], 10)

    def test_duplicate_simulation_observations_cannot_mask_stall(self):
        budget = SimulationBudget(0, 0, 10, 400)
        self.assertIsNone(budget.reason(0, 29))
        self.assertEqual(budget.reason(0, 30), "simulation_stalled")

    def test_progress_cannot_remove_absolute_wall_limit_or_run_clocks_backward(self):
        budget = SimulationBudget(0, 0, 100, 40)
        self.assertIsNone(budget.reason(1, 20))
        with self.assertRaises(ValueError):
            budget.reason(1, 19)
        self.assertEqual(budget.reason(2, 40), "wall_budget_exhausted")
        with self.assertRaises(ValueError):
            budget.reason(1, 41)
