"""Separate bounded compute time from simulated physical task durations."""

import math


def dwell_wall_budget(duration):
    """Allow slow force-PID scenes without shortening any physical observation.

    HD loaded seating measured 130 wall seconds for only 2.402 simulated
    seconds. A finite 120:1 compute allowance plus startup margin leaves room
    for the additional nest camera. No-progress detection remains 30 seconds.
    """
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("Positive finite physical observation duration required")
    return max(240.0, 60.0 + 120.0 * duration)


class SimulationBudget:
    def __init__(self, sim_start, wall_start, simulation_s, wall_s, stall_s=30):
        if (
            not all(map(math.isfinite, (sim_start, wall_start, simulation_s, wall_s, stall_s)))
            or min(simulation_s, wall_s, stall_s) <= 0
        ):
            raise ValueError("Finite timestamps and positive execution budgets required")
        self.sim_start, self.wall_start = sim_start, wall_start
        self.last_sim, self.last_progress_wall = sim_start, wall_start
        self.last_wall = wall_start
        self.summary = {
            "simulation_budget_s": simulation_s,
            "wall_budget_s": wall_s,
            "stall_budget_wall_s": stall_s,
        }

    def reason(self, sim_now, wall_now):
        if (
            not math.isfinite(sim_now)
            or not math.isfinite(wall_now)
            or sim_now < self.last_sim
            or wall_now < self.last_wall
        ):
            raise ValueError("Execution clocks must be finite and nondecreasing")
        self.last_wall = wall_now
        if sim_now > self.last_sim:
            self.last_sim, self.last_progress_wall = sim_now, wall_now
        self.summary.update(
            elapsed_simulation_s=sim_now - self.sim_start, elapsed_wall_s=wall_now - self.wall_start
        )
        reason = (
            "simulation_budget_exhausted"
            if sim_now - self.sim_start >= self.summary["simulation_budget_s"]
            else "wall_budget_exhausted"
            if wall_now - self.wall_start >= self.summary["wall_budget_s"]
            else "simulation_stalled"
            if wall_now - self.last_progress_wall >= self.summary["stall_budget_wall_s"]
            else None
        )
        self.summary["stop_reason"] = reason
        return reason
