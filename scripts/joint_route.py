"""Bounded deterministic bidirectional RRT for screened joint-space transit."""

import numpy as np


def joint_route(start, goal, limits, valid, *, seed=0, max_nodes=800, step=0.25, resolution=0.01):
    start, goal, limits = (np.asarray(x, dtype=float) for x in (start, goal, limits))
    if (
        start.ndim != 1
        or goal.shape != start.shape
        or limits.shape != (len(start), 2)
        or not np.isfinite([start, goal]).all()
        or not np.isfinite(limits).all()
        or np.any(limits[:, 0] >= limits[:, 1])
        or max_nodes < 2
        or not np.isfinite([step, resolution]).all()
        or min(step, resolution) <= 0
    ):
        raise ValueError("Finite joints, limits and positive search bounds required")
    checks = 0

    def state(q):
        nonlocal checks
        checks += 1
        return bool(np.all(q >= limits[:, 0]) and np.all(q <= limits[:, 1]) and valid(q))

    def edge(a, b):
        count = max(1, int(np.ceil(np.max(np.abs(b - a)) / resolution)))
        return all(state(a + (b - a) * t) for t in np.linspace(0, 1, count + 1))

    def result(path, reason=None):
        return {
            "success": path is not None,
            "path": None if path is None else np.asarray(path).tolist(),
            "reason": reason,
            "state_checks": checks,
            "seed": seed,
            "resolution_rad": resolution,
            "maximum_nodes": max_nodes,
            "scope": "Discrete static-geometry joint screening; no self/cable/object collision or execution guarantee",
        }

    if not state(start) or not state(goal):
        return result(None, "Start or goal rejected")
    if edge(start, goal):
        return result([start, goal])
    rng = np.random.default_rng(seed)
    trees = [{"q": [start], "parent": [-1]}, {"q": [goal], "parent": [-1]}]

    def extend(tree, target):
        index = int(np.argmin([np.linalg.norm(q - target) for q in tree["q"]]))
        q = tree["q"][index]
        distance = np.linalg.norm(target - q)
        if distance < 1e-9:
            return index, True
        candidate = q + (target - q) * min(1, step / distance)
        if not edge(q, candidate):
            return None, False
        tree["q"].append(candidate)
        tree["parent"].append(index)
        return len(tree["q"]) - 1, distance <= step

    def trace(tree, index):
        path = []
        while index >= 0:
            path.append(tree["q"][index])
            index = tree["parent"][index]
        return path[::-1]

    for turn in range(max_nodes * 4):
        if sum(len(t["q"]) for t in trees) >= max_nodes:
            break
        a = turn % 2
        target = (
            trees[1 - a]["q"][0] if rng.random() < 0.15 else rng.uniform(limits[:, 0], limits[:, 1])
        )
        index, _ = extend(trees[a], target)
        if index is None:
            continue
        target = trees[a]["q"][index]
        while sum(len(t["q"]) for t in trees) < max_nodes:
            other, reached = extend(trees[1 - a], target)
            if other is None:
                break
            if reached:
                halves = {a: trace(trees[a], index), 1 - a: trace(trees[1 - a], other)}
                path = halves[0] + halves[1][-2::-1]
                # Shorten only when the entire replacement edge is screened.
                i = 0
                while i < len(path) - 2:
                    for j in range(len(path) - 1, i + 1, -1):
                        if edge(path[i], path[j]):
                            path[i + 1 : j] = []
                            break
                    i += 1
                if not all(edge(x, y) for x, y in zip(path[:-1], path[1:])):
                    return result(None, "Final route verification rejected")
                return result(path)
    return result(None, "Search budget exhausted")
