"""Synthetic one-off board designs and randomized multi-connector harness trials."""

import math
import random

from task_schema import number

COLORS = ["0.9 0.25 0.08 1", "0.1 0.35 0.9 1", "0.8 0.65 0.05 1"]


def make_case(design_seed, placement_seed):
    design_rng = random.Random(design_seed)
    placement_rng = random.Random(placement_seed)
    width, height = design_rng.randrange(180, 221, 10), design_rng.randrange(160, 201, 10)
    sites = [(x, y) for x in (0.25, 0.75) for y in (0.25, 0.75)]
    design_rng.shuffle(sites)
    targets = list(range(3))
    design_rng.shuffle(targets)
    sockets = [
        {
            "id": f"J{i + 1}",
            "component": "wire_harness",
            "x_mm": round(width * x) + design_rng.randint(-4, 4),
            "y_mm": round(height * y) + design_rng.randint(-4, 4),
            "yaw_deg": design_rng.choice([0, 90]),
            "color": COLORS[targets.index(i)],
        }
        for i, (x, y) in enumerate(sites[:3])
    ]
    design = {
        "drawing_id": f"D{design_seed}",
        "board_id": f"B{design_seed}",
        "harness_id": "WH1",
        "origin_m": [0.16, 0.08, 0.14],
        "size_mm": [width, height],
        "thickness_mm": 20,
        "hole_width_mm": 8,
        "pin_spacing_mm": 20,
        "receptacle_depth_mm": 40,
        "sockets": sockets,
        "connections": [
            {"plug_id": f"P{i + 1}", "socket": f"J{target + 1}"} for i, target in enumerate(targets)
        ],
    }
    # Broadly separated branches avoid intersecting geometry at initialization.
    # Random translation, rotation, branch directions and lengths change all plug poses.
    trial = {
        "placement_seed": placement_seed,
        "pose_m_rad": [
            -0.30 + placement_rng.uniform(-0.025, 0.025),
            0.05 + placement_rng.uniform(-0.02, 0.02),
            0.177,
            0,
            0,
            placement_rng.uniform(-math.pi, math.pi),
        ],
        "branches": [
            {
                "plug_id": f"P{i + 1}",
                "segments": 9,
                "angle_rad": i * 2 * math.pi / 3 + placement_rng.uniform(-0.15, 0.15),
                "curve_rad": placement_rng.uniform(-0.5, 0.5),
                "color": COLORS[i],
            }
            for i in range(3)
        ],
    }
    return {
        "schema_version": 2,
        "design_seed": design_seed,
        "design": design,
        "simulation_initialization": trial,
    }


def validate_connection_plan(plan, design):
    """Reject incomplete, duplicated or inconsistent model predictions; never fill them in."""
    if not isinstance(plan, dict) or any(
        plan.get(key) != design[key] for key in ("drawing_id", "board_id", "harness_id")
    ):
        raise ValueError("Drawing, board and harness IDs must match the current design")
    rows = plan.get("connections")
    expected = {row["plug_id"]: row["socket"] for row in design["connections"]}
    if not isinstance(rows, list) or len(rows) != len(expected):
        raise ValueError("Every harness connector must be specified exactly once")
    seen_plugs, seen_sockets = set(), set()
    sockets = {row["id"]: row for row in design["sockets"]}
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("Connection must be an object")
        plug, socket = row.get("plug_id"), row.get("socket")
        if not isinstance(plug, str) or not isinstance(socket, str):
            raise ValueError("Connector IDs must be strings")
        if (
            plug in seen_plugs
            or socket in seen_sockets
            or plug not in expected
            or expected[plug] != socket
        ):
            raise ValueError("Duplicate or incorrect plug-to-socket mapping")
        target = sockets[socket]
        if any(
            not number(row.get(key)) or row[key] != target[key]
            for key in ("x_mm", "y_mm", "yaw_deg")
        ):
            raise ValueError("Predicted coordinates/orientation disagree with the design")
        seen_plugs.add(plug)
        seen_sockets.add(socket)
    return rows
