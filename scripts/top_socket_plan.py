"""Plan a regrasped connector to its drawing-selected socket; never actuates."""

import math

import numpy as np
from arm_collision_bounds import FingerStateBounds, first_overlap, screen_paths, with_parked_arms
from assembly_case import validate_connection_plan
from carried_connector import CarriedConnector
from cooperative_transfer import pose_samples, rigid_pose, socket_poses
from fixture_collision_bounds import FixtureBounds
from joint_route import joint_route


def precise_path(arm, start, goal):
    path = [np.asarray(start, dtype=float)]
    for pose in pose_samples(arm.forward(start), goal, spacing=0.0025, angular_spacing=0.005)[1:]:
        result = arm.inverse(
            pose, seed=path[-1], attempts=1, position_tolerance=0.0001, angle_tolerance=0.002
        )
        q = np.asarray(result["q_rad"], dtype=float)
        if (
            not result["success"]
            or q.shape != path[-1].shape
            or not np.isfinite(q).all()
            or np.max(np.abs(q - path[-1])) > 0.15
        ):
            raise ValueError("Precise insertion waypoint unreachable or discontinuous")
        path.append(q)
    return np.asarray(path)


def plan_socket(
    arms, bounds, world, current, primary, board, drawing_plan, plug, connector_from_tool
):
    if (
        len(arms) != 2
        or set(arms) != set(bounds)
        or set(arms) != set(current)
        or primary not in arms
    ):
        raise ValueError("Both measured arm states and collision models required")
    rows = validate_connection_plan(drawing_plan, board)
    matching = [r for r in rows if r["plug_id"] == plug]
    if len(matching) != 1:
        raise ValueError("Exactly one drawing row required for the connector")
    row = matching[0]
    relation = rigid_pose(connector_from_tool)
    targets = socket_poses(board, row, relation)
    seated_body = targets["seated"] @ np.linalg.inv(relation)
    seating = {"socket_id": row["socket"], "connector_pose": seated_body.tolist()}
    fixtures = FixtureBounds(world, include_carrier=True)
    part = CarriedConnector(world, plug)
    full = {}
    for name, bound in bounds.items():
        while isinstance(bound, FingerStateBounds):
            bound = bound.bounds
        fingers = {
            j.get("name"): [-0.0005, 0.0205] for j in bound.joints if j.get("type") == "prismatic"
        }
        if len(fingers) != 2:
            raise ValueError("Two known mechanical finger strokes required")
        full[name] = FingerStateBounds(bound, fingers)
    parked = {n: np.asarray(q, dtype=float) for n, q in current.items() if n != primary}
    arm = arms[primary]
    direction = targets["hover"][:2, 3] - arm.base[:2, 3]
    waist = math.atan2(direction[1], direction[0]) - math.atan2(arm.base[1, 0], arm.base[0, 0])
    waist = (waist + math.pi) % (2 * math.pi) - math.pi
    seeds = [
        np.asarray(current[primary]),
        np.array([waist, 1.2, 1, 0, 1, 0]),
        np.array([waist, 1.2, 1, math.pi, -1, 0]),
    ]
    seeds += list(
        np.random.default_rng(0).uniform(arm.limits[:, 0] * 0.7, arm.limits[:, 1] * 0.7, (8, 6))
    )
    report = {
        "success": False,
        "attempts": [],
        "drawing_row": row,
        "goal_connector_pose": seated_body.tolist(),
        "scope": "Static planning after an independently verified physical top regrasp. Full finger strokes, arm/fixture boxes and carried housing/pins checked. No cable collision, contact, loaded motion or insertion success proof",
    }
    for index, seed in enumerate(seeds):
        attempt = {"seed_index": index, "success": False}
        report["attempts"].append(attempt)
        try:
            result = arm.inverse(
                targets["hover"],
                seed=seed,
                attempts=1,
                position_tolerance=0.0001,
                angle_tolerance=0.002,
            )
            if not result["success"]:
                raise ValueError("Socket hover IK failed")

            def valid(q):
                state = {primary: q, **parked}
                return (
                    first_overlap(full, state) is None
                    and fixtures.first_overlap(full, state, margin=0.001) is None
                    and part.fixture_overlap(arm.forward(q) @ np.linalg.inv(relation), fixtures)[0]
                    is None
                )

            route = joint_route(current[primary], result["q_rad"], arm.limits, valid, max_nodes=600)
            attempt["route"] = route
            if not route["success"]:
                raise ValueError("Loaded socket transit rejected: " + str(route["reason"]))
            transit = np.asarray(route["path"])
            descent = precise_path(arm, transit[-1], targets["seated"])
            paths = {"hover": transit, "insert": descent}
            attempt["checks"] = {
                phase: {
                    "arms": screen_paths(full, with_parked_arms({primary: path}, parked)),
                    "fixtures": fixtures.screen(
                        full, with_parked_arms({primary: path}, parked), margin=0.001
                    ),
                    "carried_connector": part.screen(
                        arm,
                        path,
                        relation,
                        fixtures,
                        seating=seating if phase == "insert" else None,
                    ),
                }
                for phase, path in paths.items()
            }
            attempt["success"] = all(
                check["clear"] for phase in attempt["checks"].values() for check in phase.values()
            )
            if attempt["success"]:
                report.update(
                    success=True,
                    selected_attempt=len(report["attempts"]) - 1,
                    connector_from_tool=relation.tolist(),
                    tool_poses={k: v.tolist() for k, v in targets.items()},
                    paths={phase: {primary: path.tolist()} for phase, path in paths.items()},
                )
                return report
        except ValueError as error:
            attempt["reason"] = str(error)
    return report
