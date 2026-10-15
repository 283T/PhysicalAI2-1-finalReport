"""Plan a visible wire pickup while the other arm remains at measured joints."""

import numpy as np
from arm_collision_bounds import with_parked_arms
from cooperative_pick import prepare_pickup_paths
from cooperative_transfer import rigid_pose
from fixture_collision_bounds import FixtureBounds


def plan_wire_pickup(
    arms, bounds, world, observation, current, support_arm, plug_id, *, half_turn=False
):
    if set(arms) != set(bounds) or set(arms) != set(current) or len(arms) != 2:
        raise ValueError("Both arms require observed joints and collision geometry")
    if support_arm not in arms or plug_id not in {"P1", "P2", "P3"}:
        raise ValueError("Known support arm and branch required")
    if (
        observation.get("frame") != "world"
        or observation.get("source") != "registered_rgbd_color_and_plane"
        or observation.get("stable_frames", 0) < 3
    ):
        raise ValueError("Three stable RGB-D wire proposals in world frame required")
    matching = [row for row in observation.get("wire_supports", []) if row["plug_id"] == plug_id]
    if len(matching) != 1:
        raise ValueError("Exactly one support proposal required for the branch")
    grip = rigid_pose(matching[0]["grasp_tcp_pose"])
    if half_turn:
        grip = grip @ np.diag([-1.0, -1.0, 1.0, 1.0])
    hover = grip.copy()
    hover[:3, 3] -= 0.10 * grip[:3, 2]
    parked = {name: np.asarray(q) for name, q in current.items() if name != support_arm}
    paths, selection = prepare_pickup_paths(
        {support_arm: arms[support_arm]},
        {support_arm: current[support_arm]},
        {support_arm: hover},
        {support_arm: grip},
        bounds,
        parked=parked,
    )
    fixtures = FixtureBounds(world)
    screens = {
        phase: fixtures.screen(bounds, with_parked_arms(path, parked), margin=0.001)
        for phase, path in paths.items()
    }
    return {
        "success": all(row["clear"] for row in screens.values()),
        "support_arm": support_arm,
        "branch": plug_id,
        "half_turn": half_turn,
        "sensor_stamp_ns": observation.get("sensor_stamp_ns"),
        "grasp_tcp_pose": grip.tolist(),
        "parked_joints_rad": {name: q.tolist() for name, q in parked.items()},
        "paths": {
            phase: {name: q.tolist() for name, q in path.items()} for phase, path in paths.items()
        },
        "branch_selection": selection,
        "fixture_preflight": screens,
        "scope": "Offline continuous IK and inter-arm/fixture box screening only; caller must verify acquisition freshness, contact, wire identity, slip and unintended collisions at execution. No self/cable/object collision guarantee",
    }
