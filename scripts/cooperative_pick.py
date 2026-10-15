"""RGB-D endpoint assignment and shared translation planning for two held terminals."""

import numpy as np
from cartesian_path import interpolate_joints, translation_path


def assign_pair(plan, plug, support):
    if plug == support:
        raise ValueError("Two different terminals are required")
    by_id = {row["plug_id"]: row for row in plan["connectors"]}
    choices = []
    for names in (("left_arm", "right_arm"), ("right_arm", "left_arm")):
        rows = [by_id[terminal] for terminal in (plug, support)]
        if any(row["grasp_candidate"] is None for row in rows):
            raise ValueError("Both terminals need measured 6D poses")
        if all(
            all(result["success"] for result in row["arms"][name].values())
            for row, name in zip(rows, names)
        ):
            cost = sum(
                np.linalg.norm(row["arms"][name]["pickup_hover"]["q_rad"])
                for row, name in zip(rows, names)
            )
            choices.append((cost, dict(zip(names, rows))))
    if not choices:
        raise ValueError("No distinct-arm assignment for both terminals")
    return min(choices, key=lambda item: item[0])[1]


def check_tool_clearance(arms, paths, minimum=0.12):
    """Screen tool origins only. This is not mesh collision checking."""
    if len(paths) != 2:
        return
    names = list(paths)
    count = max(len(path) for path in paths.values()) * 10
    for fraction in np.linspace(0, 1, max(2, count)):
        points = [
            arms[name].forward(interpolate_joints(paths[name], fraction))[:3, 3] for name in names
        ]
        if np.linalg.norm(points[0] - points[1]) < minimum:
            raise ValueError("Tool-origin separation below conservative 120 mm screen")


def shared_lift(arms, current, height=0.10):
    """Both tools move by the same world-Z displacement, preserving endpoint spacing."""
    if not np.isfinite(height) or height <= 0:
        raise ValueError("Lift height must be finite and positive")
    targets, paths = {}, {}
    for name, q in current.items():
        target = arms[name].forward(q)
        target[2, 3] += height
        targets[name] = target
        paths[name] = translation_path(arms[name], q, target)
    check_tool_clearance(arms, paths)
    return targets, paths


def assess_cooperation(closed, samples):
    """Measure endpoint spacing against the closed state during the hold window."""
    names = list(closed["arms"])
    if len(names) != 2 or len(samples) < 10:
        return {"success": False, "reason": "Need two arms and ten shared observations"}
    a, b = names
    reference = np.asarray(closed["arms"][b]["plug"]) - closed["arms"][a]["plug"]
    offsets = [np.asarray(row["arms"][b]["plug"]) - row["arms"][a]["plug"] for row in samples]
    shift = float(max(np.linalg.norm(offset - reference) for offset in offsets))
    separations = [
        np.linalg.norm(np.asarray(row["arms"][b]["tcp"]) - row["arms"][a]["tcp"]) for row in samples
    ]
    minimum = float(min(separations))
    return {
        "success": shift < 0.005 and minimum >= 0.12,
        "maximum_endpoint_relative_shift_m": shift,
        "minimum_tool_separation_m": minimum,
        "reference_endpoint_offset_m": reference.tolist(),
    }


def upright_waist_path(current, goal):
    """Turn the waist while the arm is upright, then bend toward the workpiece."""
    current, goal = np.asarray(current, dtype=float), np.asarray(goal, dtype=float)
    if current.shape != (6,) or goal.shape != (6,) or not np.isfinite([current, goal]).all():
        raise ValueError("Waist preparation needs finite six-joint vectors")
    if np.max(np.abs(current[1:])) > 0.02:
        raise ValueError("Waist-first approach requires a fresh upright scene")
    waypoint = current.copy()
    waypoint[0] = goal[0]
    return np.asarray([current, waypoint, goal])


class PickupPathError(ValueError):
    def __init__(self, message, diagnostics):
        super().__init__(message)
        self.diagnostics = diagnostics


def prepare_pickup_paths(
    arms,
    current,
    hovers,
    grips,
    bounds=None,
    *,
    parked=None,
    include_lift=True,
    precise_endpoints=False,
    transit_router=None,
):
    """Select IK branches with contiguous descent/lift before moving any arm."""
    import itertools
    import math

    from arm_collision_bounds import screen_paths, with_parked_arms
    from cooperative_transfer import single_arm_path

    choices = {}
    rejected = []
    for name, arm in arms.items():
        direction = hovers[name][:2, 3] - arm.base[:2, 3]
        waist = math.atan2(direction[1], direction[0]) - math.atan2(arm.base[1, 0], arm.base[0, 0])
        waist = (waist + math.pi) % (2 * math.pi) - math.pi
        seeds = [
            np.array([waist, 1.2, 1.0, 0, 1.0, 0]),
            np.array([waist, 1.2, 1.0, math.pi, -1.0, 0]),
            np.array([waist, 1.4, 0.8, 0, 1.2, 0]),
            np.asarray(current[name]),
        ]
        choices[name] = []
        for seed_index, seed in enumerate(seeds):
            result = arm.inverse(hovers[name], seed=seed, attempts=1)
            if not result["success"]:
                rejected.append({"arm": name, "seed": seed_index, "reason": "Hover IK failed"})
                continue
            try:
                hover_q = np.asarray(result["q_rad"])
                descent = (single_arm_path if precise_endpoints else translation_path)(
                    arm, hover_q, grips[name]
                )
                lift = None
                if include_lift:
                    lift_target = arm.forward(descent[-1])
                    lift_target[2, 3] += 0.10
                    lift = translation_path(arm, descent[-1], lift_target)
                hover_path = (
                    transit_router(name, current[name], hover_q)
                    if transit_router is not None
                    else upright_waist_path(current[name], hover_q)
                )
                choices[name].append(
                    {
                        "hover": hover_path,
                        "approach": descent,
                        "lift": lift,
                        "seed_index": seed_index,
                    }
                )
            except ValueError as error:
                rejected.append({"arm": name, "seed": seed_index, "reason": str(error)})
        if not choices[name]:
            raise PickupPathError(
                "No continuous pickup branch for " + name, {"individual": rejected}
            )
    names = list(arms)
    combinations = []
    for combination in itertools.product(*(choices[name] for name in names)):
        paths = {
            phase: {name: choice[phase] for name, choice in zip(names, combination)}
            for phase in (("hover", "approach", "lift") if include_lift else ("hover", "approach"))
        }
        screens = (
            {
                phase: screen_paths(bounds, with_parked_arms(path, parked or {}))
                for phase, path in paths.items()
            }
            if bounds
            else {}
        )
        combinations.append(
            {
                "seeds": {name: choice["seed_index"] for name, choice in zip(names, combination)},
                "screens": screens,
            }
        )
        if all(screen["clear"] for screen in screens.values()):
            return paths, {
                "seeds": {name: choice["seed_index"] for name, choice in zip(names, combination)},
                "screens": screens,
                "rejected_individual_candidates": rejected,
            }
    raise PickupPathError(
        "No continuous IK branches passed active/parked arm box screening",
        {"individual": rejected, "combinations": combinations},
    )
