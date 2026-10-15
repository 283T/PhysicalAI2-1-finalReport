"""Plan a visible wire pickup while the other arm remains at measured joints."""

import numpy as np
from arm_collision_bounds import with_parked_arms
from arm_kinematics import axis_rotation
from cooperative_pick import prepare_pickup_paths
from cooperative_transfer import rigid_pose
from fixture_collision_bounds import FixtureBounds


def upright_wire_target(proposal):
    """Vertical fingers clear the table despite a rolled square wire face."""
    measured = rigid_pose(proposal["grasp_tcp_pose"])
    tangent, normal = measured[:3, 1], -measured[:3, 2]
    horizontal = tangent.copy()
    horizontal[2] = 0
    if np.linalg.norm(horizontal) < 0.95:
        raise ValueError("Tabletop wire tangent must be nearly horizontal")
    horizontal /= np.linalg.norm(horizontal)
    up = np.array([0.0, 0.0, 1.0])
    jaw = np.cross(up, horizontal)
    tool = np.eye(4)
    tool[:3, :3] = np.column_stack((jaw, horizontal, -up))
    tool[:3, 3] = np.asarray(proposal["wire_center_xyz_m"]) - [0, 0, 0.001]
    width = 0.008 * (abs(np.dot(jaw, normal)) + abs(np.dot(jaw, np.cross(tangent, normal))))
    return tool, float(width)


def plan_wire_pickup(
    arms,
    bounds,
    world,
    observation,
    current,
    support_arm,
    plug_id,
    *,
    half_turn=False,
    include_lift=True,
    require_stable=True,
    upright=False,
    include_carrier=False,
    tip_depth_m=None,
    face_quarter_turns=0,
):
    if face_quarter_turns not in (0, 1, 2, 3) or (upright and face_quarter_turns):
        raise ValueError("A square-wire face index 0..3 requires the full wire frame")
    if tip_depth_m is not None and (not np.isfinite(tip_depth_m) or not 0 < tip_depth_m <= 0.012):
        raise ValueError("Finite positive tip depth up to 12 mm required")
    if set(arms) != set(bounds) or set(arms) != set(current) or len(arms) != 2:
        raise ValueError("Both arms require observed joints and collision geometry")
    if support_arm not in arms or plug_id not in {"P1", "P2", "P3"}:
        raise ValueError("Known support arm and branch required")
    if (
        observation.get("frame") != "world"
        or observation.get("source") != "registered_rgbd_color_and_plane"
        or (require_stable and observation.get("stable_frames", 0) < 3)
    ):
        raise ValueError("Three stable RGB-D wire proposals in world frame required")
    matching = [row for row in observation.get("wire_supports", []) if row["plug_id"] == plug_id]
    if len(matching) != 1:
        raise ValueError("Exactly one support proposal required for the branch")
    grip, expected_width = (
        upright_wire_target(matching[0])
        if upright
        else (rigid_pose(matching[0]["grasp_tcp_pose"]), 0.008)
    )
    if face_quarter_turns:
        grip[:3, :3] = grip[:3, :3] @ axis_rotation([0, 1, 0], face_quarter_turns * np.pi / 2)
    if tip_depth_m is not None or face_quarter_turns:
        depth = 0.002 if tip_depth_m is None else tip_depth_m
        grip[:3, 3] = np.asarray(matching[0]["wire_center_xyz_m"]) + grip[:3, 2] * depth
        grip = rigid_pose(grip)
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
        include_lift=include_lift,
        precise_endpoints=True,
    )
    fixtures = FixtureBounds(world, include_carrier=include_carrier)
    screens = {
        phase: fixtures.screen(bounds, with_parked_arms(path, parked), margin=0.001)
        for phase, path in paths.items()
    }
    return {
        "success": all(row["clear"] for row in screens.values()),
        "support_arm": support_arm,
        "branch": plug_id,
        "half_turn": half_turn,
        "face_quarter_turns": face_quarter_turns,
        "upright": upright,
        "expected_wire_width_m": expected_width,
        "tip_depth_m": tip_depth_m if tip_depth_m is not None else 0.001 if upright else 0.002,
        "includes_independent_support_lift": include_lift,
        "stable_observation_required": require_stable,
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
