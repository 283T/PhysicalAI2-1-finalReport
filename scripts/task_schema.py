"""Validate assembly instructions against the calibrated board fixture."""

import math


def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def validate_task(task, board):
    if not isinstance(task, dict):
        raise ValueError("Task must be an object")
    if not isinstance(task.get("source"), str) or not task["source"]:
        raise ValueError("Task source is required")
    origin = task.get("board_origin_m")
    if (
        not isinstance(origin, list)
        or len(origin) != 3
        or not all(number(value) for value in origin)
        or origin != board["origin_m"]
    ):
        raise ValueError("Board origin must match the calibrated fixture")
    if not all(number(task.get(key)) for key in ("x_mm", "y_mm", "yaw_deg")):
        raise ValueError("Coordinates and orientation must be finite numbers")
    if task.get("harness_id") != board["harness_id"] or task.get("plug_id") != board["plug_id"]:
        raise ValueError("Harness and plug must match the registered assembly")
    matching = [
        socket
        for socket in board["sockets"]
        if all(task.get(key) == socket[key] for key in ("component", "x_mm", "y_mm", "yaw_deg"))
    ]
    if not matching or (task.get("socket") is not None and task["socket"] != matching[0]["id"]):
        raise ValueError("Component, position and orientation must match a registered socket")
    return matching[0]
