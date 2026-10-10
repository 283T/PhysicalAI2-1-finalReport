"""Match validated drawing rows to camera detections and calculate geometric grasp/hover endpoint IK."""

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from arm_kinematics import ArmKinematics, transform
from assembly_case import validate_connection_plan
from grasp_poses import candidate_poses


def load_arms(world_path, model_directory):
    world = ET.parse(world_path).getroot().find("world")
    arms = {}
    for name in ("left_arm", "right_arm"):
        pose = [float(v) for v in world.findtext(f"model[@name='{name}']/pose").split()]
        arms[name] = ArmKinematics(model_directory / f"{name}.urdf", name + "_gripper", pose)
    return arms


def plan_reach(plan, board, detections, arms):
    rows = validate_connection_plan(plan, board)
    if detections.get("frame") != "world":
        raise ValueError("Connector observations must use world frame")
    observed = detections.get("connectors", [])
    if len(observed) != len(rows) or {d["plug_id"] for d in observed} != {
        r["plug_id"] for r in rows
    }:
        raise ValueError("Need exactly one observation per drawing connector")
    by_id = {row["plug_id"]: row for row in observed}
    results = []
    for row in rows:
        observation = by_id[row["plug_id"]]
        position = np.asarray(observation["surface_xyz_m"], dtype=float)
        yaw = observation["yaw_rad"]
        if position.shape != (3,) or not np.isfinite(position).all() or not math.isfinite(yaw):
            raise ValueError("Observation pose must be finite")
        hover = position + np.array([0, 0, 0.10])
        target = transform(hover, [math.pi, 0, yaw])
        goal = np.asarray(board["origin_m"]) + np.array(
            [row["x_mm"] / 1000, row["y_mm"] / 1000, 0.16]
        )
        socket_target = transform(goal, [math.pi, 0, math.radians(row["yaw_deg"])])
        grasp = None
        if "pose_6d" in observation:
            grasp = candidate_poses(observation, goal, row["yaw_deg"])
            target = np.asarray(grasp["pickup_hover_pose"])
            socket_target = np.asarray(grasp["socket_hover_pose"])
        options = {
            name: {"pickup_hover": arm.inverse(target), "socket_hover": arm.inverse(socket_target)}
            for name, arm in arms.items()
        }
        if grasp is not None:
            for name, arm in arms.items():
                options[name]["pickup_grasp"] = arm.inverse(
                    np.asarray(grasp["grasp_tcp_pose"]), seed=options[name]["pickup_hover"]["q_rad"]
                )
        available = [
            name for name, poses in options.items() if all(p["success"] for p in poses.values())
        ]
        selected = (
            min(available, key=lambda name: np.linalg.norm(options[name]["pickup_hover"]["q_rad"]))
            if available
            else None
        )
        results.append(
            {
                "plug_id": row["plug_id"],
                "socket": row["socket"],
                "pickup_hover_pose": target.tolist(),
                "socket_hover_pose": socket_target.tolist(),
                "selected_arm": selected,
                "grasp_candidate": grasp,
                "arms": options,
            }
        )
    return {
        "drawing_id": plan["drawing_id"],
        "board_id": plan["board_id"],
        "sensor_stamp_ns": detections["sensor_stamp_ns"],
        "frame": "world",
        "connectors": results,
        "success": all(row["selected_arm"] is not None for row in results),
        "scope": (
            "Coded-face geometric grasp/hover candidates preserving connector-to-tool transform; no contact, collision path or holding validation"
            if all(row["grasp_candidate"] is not None for row in results)
            else "Offline endpoint reachability with assumed downward tool pose; no collision path, 6D grasp, cable constraints or execution authorization"
        ),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--board", type=Path, required=True)
    parser.add_argument("--detections", type=Path, required=True)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--models", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.unlink(missing_ok=True)
    result = plan_reach(
        *(json.loads(path.read_text()) for path in (args.plan, args.board, args.detections)),
        load_arms(args.world, args.models),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    if not result["success"]:
        raise RuntimeError("Some terminals have no common pickup/socket arm solution")


if __name__ == "__main__":
    main()
