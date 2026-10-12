"""Contact pickup of one or two RGB-D-derived terminals with MELFA arms."""

import argparse
import hashlib
import json
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from arm_collision_bounds import ArmCollisionBounds, screen_paths
from arm_contacts import CONTACT_TOPIC, accumulate_contact, inter_arm_pair
from arm_kinematics import quaternion_rotation, rotation_error
from cartesian_path import interpolate_joints, translation_path
from cooperative_pick import (
    assess_cooperation,
    assign_pair,
    check_tool_clearance,
    prepare_pickup_paths,
    shared_lift,
    upright_waist_path,
)
from fixture_collision_bounds import FixtureBounds
from grasp_validation import assess_hold, assess_relative_grasp, assess_seat, world_link_positions
from gripper_model import OPEN_TRAVEL_M
from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
from plan_nest_transfer import plan_to_nest
from plan_reach import load_arms, plan_reach
from processes import stop_process
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--board", type=Path, required=True)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--models", type=Path, default=Path("outputs/melfa_worlds"))
    parser.add_argument("--plug", choices=("P1", "P2", "P3"), default="P1")
    parser.add_argument(
        "--support-plug",
        choices=("P1", "P2", "P3"),
        help="Hold a second terminal with the other arm and lift together",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--place-in-nest",
        action="store_true",
        help="After paired pickup, plan from live joints, place the primary terminal in the passive nest and retreat",
    )
    parser.add_argument(
        "--keep-camera",
        action="store_true",
        help="Keep full RGB-D rendering rate after the accepted static observation",
    )
    args = parser.parse_args()
    if args.place_in_nest and not args.support_plug:
        parser.error("--place-in-nest requires --support-plug")
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "validation.json").unlink(missing_ok=True)
    observations = args.output / "observations"
    # Capture from the currently running scene; never replay a previous RGB-D snapshot.
    subprocess.run(
        [
            "python3",
            "scripts/observe_connectors.py",
            "--bridge",
            "--require-6d",
            "--output",
            str(observations),
        ],
        check=True,
    )
    detections = json.loads((observations / "detections.json").read_text())
    arms = load_arms(args.world, args.models)
    plan = plan_reach(
        json.loads(args.plan.read_text()), json.loads(args.board.read_text()), detections, arms
    )
    (args.output / "grasp_plan.json").write_text(json.dumps(plan, indent=2) + "\n")
    row = next(r for r in plan["connectors"] if r["plug_id"] == args.plug)
    if not row["selected_arm"] or not row["grasp_candidate"]:
        raise ValueError("Selected terminal has no complete 6D grasp/hover solution")
    assigned = (
        assign_pair(plan, args.plug, args.support_plug)
        if args.support_plug
        else {row["selected_arm"]: row}
    )
    selected = {name: arms[name] for name in assigned}
    collision_bounds = (
        {
            name: ArmCollisionBounds(args.models / (name + ".urdf"), arm)
            for name, arm in selected.items()
        }
        if len(selected) == 2
        else None
    )
    report = {
        "success": False,
        "plug_id": args.plug,
        "support_plug": args.support_plug,
        "assignment": {name: row["plug_id"] for name, row in assigned.items()},
        "scope": "RGB-D-derived contact pickup; optional second-terminal support, not direct wire support, collision planning or insertion",
        "world_sha256": hashlib.sha256(args.world.read_bytes()).hexdigest(),
    }
    sensor = ET.parse(args.world).getroot().find("world/model[@name='overhead_camera']/link/sensor")
    camera_service = sensor.findtext("topic") + "/set_rate"
    original_rate = float(sensor.findtext("update_rate"))
    throttled = False

    def set_camera_rate(rate):
        subprocess.run(
            [
                "gz",
                "service",
                "-s",
                camera_service,
                "--reqtype",
                "gz.msgs.Double",
                "--req",
                f"data: {rate}",
                "--timeout",
                "2000",
            ],
            check=True,
            capture_output=True,
            timeout=5,
        )

    rclpy.init()
    node = rclpy.create_node("melfa_pick_connector")
    transport = Node()
    truth = []
    pose_topic = "/world/multi_connector_factory/dynamic_pose/info"

    def receive(msg):
        poses = {
            p.name: {
                "xyz": [p.position.x, p.position.y, p.position.z],
                "q": [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w],
            }
            for p in msg.pose
        }
        needed = ["wire_harness_multi", "P1", "P2", "P3"]
        needed += [key for name in selected for key in (name, name + "_rv4frl_hand_flange")]
        if not all(key in poses for key in needed):
            return
        tools = {}
        for name, arm in selected.items():
            flange = name + "_rv4frl_hand_flange"
            model_rotation = quaternion_rotation(poses[name]["q"])
            rotation = model_rotation @ quaternion_rotation(poses[flange]["q"])
            position = (
                world_link_positions(poses, name, [flange])[flange] + rotation @ arm.tool[:3, 3]
            )
            tools[name] = (position, rotation)
        plugs = world_link_positions(poses, "wire_harness_multi", ["P1", "P2", "P3"])
        cable_names = [key for key in poses if "_wire_" in key]
        cable = world_link_positions(poses, "wire_harness_multi", cable_names)
        sim_time = msg.header.stamp.sec + msg.header.stamp.nsec / 1e9
        rotations = {
            plug: quaternion_rotation(poses["wire_harness_multi"]["q"])
            @ quaternion_rotation(poses[plug]["q"])
            for plug in ("P1", "P2", "P3")
        }
        truth[:] = [(time.monotonic(), tools, plugs, cable, sim_time, rotations)]

    transport.subscribe(Pose_V, pose_topic, receive)
    contacts = []
    contact_diagnostics = {}
    current_phase = ["initialization"]
    contact_updates = [0]

    def receive_contacts(msg):
        contact_updates[0] += 1
        for contact in msg.contact:
            first, second = contact.collision1.name, contact.collision2.name
            accumulate_contact(
                contact_diagnostics,
                current_phase[0],
                first,
                second,
                contact.depth,
                msg.header.stamp.sec + msg.header.stamp.nsec / 1e9,
            )
            if inter_arm_pair(first, second):
                contacts.append(
                    {
                        "collision1": first,
                        "collision2": second,
                        "depth_m": list(contact.depth),
                        "sim_time_s": msg.header.stamp.sec + msg.header.stamp.nsec / 1e9,
                    }
                )
                del contacts[20:]

    transport.subscribe(Contacts, CONTACT_TOPIC, receive_contacts)

    def receive_joints(name, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(key in positions for key in selected[name].names):
            joints[name] = (time.monotonic(), positions)

    joints = {}
    pubs = {}
    bridge_command = ["ros2", "run", "ros_gz_bridge", "parameter_bridge"]
    for name in selected:
        node.create_subscription(
            JointState,
            f"/{name}/joints",
            lambda msg, name=name: receive_joints(name, msg),
            qos_profile_sensor_data,
        )
        topics = [f"/{name}/joint_{i}/cmd" for i in range(6)] + [
            f"/{name}/gripper/{f}/cmd" for f in ("left", "right")
        ]
        pubs[name] = [node.create_publisher(Float64, topic, 10) for topic in topics]
        bridge_command.append(f"/{name}/joints@sensor_msgs/msg/JointState[gz.msgs.Model")
        bridge_command += [topic + "@std_msgs/msg/Float64]gz.msgs.Double" for topic in topics]
    bridge = None
    history = []
    last_saved = [-1.0, -1.0]
    target_q = None
    target_grip = {name: OPEN_TRAVEL_M for name in selected}
    transport_reference = None
    fixtures = FixtureBounds(args.world) if args.place_in_nest else None

    def sample():
        if (
            not truth
            or any(name not in joints for name in selected)
            or min([truth[0][0]] + [joints[name][0] for name in selected]) < time.monotonic() - 0.5
        ):
            return None
        states = {}
        for name, arm in selected.items():
            positions = joints[name][1]
            finger_names = [name + "_finger_" + f + "_joint" for f in ("left", "right")]
            if not all(key in positions for key in finger_names):
                return None
            states[name] = {
                "plug_id": assigned[name]["plug_id"],
                "plug": truth[0][2][assigned[name]["plug_id"]].tolist(),
                "tcp": truth[0][1][name][0].tolist(),
                "fingers": [positions[key] for key in finger_names],
                "q_rad": [positions[key] for key in arm.names],
                "plug_rotation_world": truth[0][5][assigned[name]["plug_id"]].tolist(),
                "tcp_rotation_world": truth[0][1][name][1].tolist(),
            }
        result = {
            "time_s": time.monotonic(),
            "sim_time_s": truth[0][4],
            "arms": states,
            "cable_z_span_m": float(np.ptp([point[2] for point in truth[0][3].values()])),
        }
        if len(states) == 1:
            result.update(next(iter(states.values())))  # Preserve single-arm trace format.
        return result

    def publish(*, allow_contact_release=False):
        if contacts and not allow_contact_release:
            raise RuntimeError("Inter-arm collision detected; see inter_arm_contacts")
        for name in selected:
            for pub, value in zip(
                pubs[name], [*target_q[name], target_grip[name], target_grip[name]]
            ):
                pub.publish(Float64(data=float(value)))
        rclpy.spin_once(node, timeout_sec=0.03)

    def record(phase):
        current_phase[0] = phase
        row = sample()
        if row is not None:
            row["phase"] = phase
            for name, arm in selected.items():
                row["arms"][name]["command_q_rad"] = target_q[name].tolist()
                row["arms"][name]["command_tcp_xyz_m"] = arm.forward(target_q[name])[:3, 3].tolist()
            if len(selected) == 1:
                row.update(next(iter(row["arms"].values())))
            if (
                not history
                or history[-1]["phase"] != phase
                or row["sim_time_s"] - last_saved[0] >= 0.02
            ):
                history.append(row)
                last_saved[0] = row["sim_time_s"]
            if row["time_s"] - last_saved[1] > 0.5:
                (args.output / "latest.json").write_text(json.dumps(row, indent=2) + "\n")
                last_saved[1] = row["time_s"]
        return row

    def dwell(phase, duration):
        current_phase[0] = phase
        print(f"{phase}: start ({duration:g} simulated seconds)", flush=True)
        rows, last = [], None
        start = truth[0][4]
        wall_start = time.monotonic()
        while truth[0][4] - start < duration:
            if time.monotonic() - wall_start > 30 + duration * 15:
                raise RuntimeError("Simulation stalled in " + phase)
            publish(allow_contact_release=phase == "final_open")
            stamp = truth[0][0] if truth else None
            if stamp != last:
                row = record(phase)
                if row:
                    rows.append(row)
                    last = stamp
        if len(rows) < 10:
            raise RuntimeError("Insufficient fresh observations in " + phase)
        return rows

    def move(phase, targets, *, straight=False, ready_paths=None, joint_motion=False):
        current_phase[0] = phase
        nonlocal target_q
        print(f"{phase}: preflight and move", flush=True)
        deadline = time.monotonic() + 5
        observation = sample()
        while observation is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            observation = sample()
        if observation is None:
            raise RuntimeError("No fresh feedback before " + phase)
        paths = {}
        duration = 3.0
        for name, arm in selected.items():
            current = np.asarray(observation["arms"][name]["q_rad"])
            target = targets[name]
            if ready_paths is not None:
                paths[name] = ready_paths[name]
                if np.max(np.abs(paths[name][0] - current)) > 0.02:
                    raise RuntimeError("Prepared path start disagrees with fresh feedback")
            elif straight:
                paths[name] = translation_path(arm, current, target)
            else:
                result = arm.inverse(target, seed=current)
                if not result["success"]:
                    raise RuntimeError("Unreachable " + phase + "/" + name)
                paths[name] = (
                    upright_waist_path(current, result["q_rad"])
                    if len(selected) == 2 and phase == "hover"
                    else np.asarray([current, result["q_rad"]])
                )
            if straight or (ready_paths is not None and not joint_motion):
                duration = max(
                    duration, np.linalg.norm(target[:3, 3] - arm.forward(current)[:3, 3]) / 0.02
                )
            else:
                duration = max(
                    duration,
                    sum(np.max(np.abs(b - a)) for a, b in zip(paths[name][:-1], paths[name][1:]))
                    / 0.5,
                )
            if args.place_in_nest and ready_paths is not None:
                duration = max(
                    duration,
                    sum(np.max(np.abs(b - a)) for a, b in zip(paths[name][:-1], paths[name][1:]))
                    / 0.35,
                )
        check_tool_clearance(selected, paths)
        if collision_bounds is not None:
            screening = screen_paths(collision_bounds, paths)
            report.setdefault("collision_preflight", []).append({"phase": phase, **screening})
            if not screening["clear"]:
                raise RuntimeError(
                    "Inter-arm enclosing boxes overlap before "
                    + phase
                    + ": "
                    + str(screening["links"])
                )
        if fixtures is not None:
            screening = fixtures.screen(collision_bounds, paths, margin=0.001)
            report.setdefault("fixture_preflight", []).append({"phase": phase, **screening})
            if not screening["clear"]:
                raise RuntimeError(
                    "Fixed fixture box rejected " + phase + ": " + str(screening["shapes"])
                )
        start = truth[0][4]
        wall_start = time.monotonic()
        stable, last = 0, None
        error = angle = float("inf")
        while truth[0][4] - start < duration + 15 and time.monotonic() - wall_start < 180:
            t = min(1, (truth[0][4] - start) / duration)
            target_q = {
                name: interpolate_joints(path, t * t * (3 - 2 * t)) for name, path in paths.items()
            }
            publish()
            observation = record(phase)
            if (
                transport_reference is not None
                and phase.startswith("nest_")
                and observation is not None
            ):
                checks = {
                    name: assess_relative_grasp(reference, observation["arms"][name])
                    for name, reference in transport_reference.items()
                }
                report["last_transport_grasp_check"] = {"phase": phase, "arms": checks}
                if not all(check["success"] for check in checks.values()):
                    raise RuntimeError("Tool-relative slip detected in " + phase)
            if observation is None or t < 1:
                continue
            valid = True
            errors = []
            for name, arm in selected.items():
                actual = arm.forward(observation["arms"][name]["q_rad"])
                measured, rotation = truth[0][1][name]
                error = np.linalg.norm(targets[name][:3, 3] - measured)
                angle = np.linalg.norm(rotation_error(targets[name][:3, :3], rotation))
                errors.append(float(error))
                valid = valid and error < 0.002 and angle < 0.025
                valid = (
                    valid
                    and np.max(np.abs(paths[name][-1] - observation["arms"][name]["q_rad"])) < 0.01
                )
                valid = valid and np.linalg.norm(actual[:3, 3] - measured) < 0.001
            if truth[0][0] != last:
                stable = stable + 1 if valid else 0
                last = truth[0][0]
            if stable >= 5:
                print(f"{phase}: maximum TCP error {max(errors) * 1000:.2f} mm", flush=True)
                return
        raise RuntimeError(
            f"{phase} did not converge: position {error:.5f} m, angle {angle:.5f} rad"
        )

    with (args.output / "control.log").open("w") as log:
        try:
            if not args.keep_camera:
                set_camera_rate(0.5)
                throttled = True
                report["camera_rate_during_motion_hz"] = 0.5
            bridge = subprocess.Popen(
                bridge_command, stdout=log, stderr=log, start_new_session=True
            )
            if len(selected) == 2:
                info = subprocess.run(
                    ["gz", "topic", "-i", "-t", CONTACT_TOPIC],
                    capture_output=True,
                    text=True,
                    check=True,
                    timeout=8,
                )
                if "gz.msgs.Contacts" not in info.stdout:
                    raise RuntimeError(
                        "Missing inter-arm contact publishers; rebuild factory world"
                    )
                report["contact_monitor_ready"] = True
            end = time.monotonic() + 30
            while sample() is None or not all(
                pub.get_subscription_count() for group in pubs.values() for pub in group
            ):
                if time.monotonic() > end:
                    raise RuntimeError("Missing MELFA/gripper feedback")
                rclpy.spin_once(node, timeout_sec=0.05)
            if contacts:
                raise RuntimeError(
                    "Inter-arm collision detected before motion; see inter_arm_contacts"
                )
            target_q = {name: np.asarray(sample()["arms"][name]["q_rad"]) for name in selected}
            for name, arm in selected.items():
                measured_fk = arm.forward(target_q[name])
                actual, rotation = truth[0][1][name]
                if (
                    np.linalg.norm(measured_fk[:3, 3] - actual) > 0.001
                    or np.linalg.norm(rotation_error(measured_fk[:3, :3], rotation)) > 0.01
                ):
                    raise RuntimeError("Current world and supplied arm kinematics disagree")
            baseline = dwell("open", 1.5)[-1]
            hovers = {
                name: np.asarray(row["grasp_candidate"]["pickup_hover_pose"])
                for name, row in assigned.items()
            }
            grips = {
                name: np.asarray(row["grasp_candidate"]["grasp_tcp_pose"])
                for name, row in assigned.items()
            }
            prepared, preparation = prepare_pickup_paths(
                selected,
                {name: np.asarray(baseline["arms"][name]["q_rad"]) for name in selected},
                hovers,
                grips,
                collision_bounds,
            )
            report["path_preparation"] = preparation
            report["prepared_joint_paths"] = {
                phase: {name: path.tolist() for name, path in paths.items()}
                for phase, paths in prepared.items()
            }
            move("hover", hovers, ready_paths=prepared["hover"], joint_motion=True)
            current = sample()
            for name in selected:
                if (
                    np.linalg.norm(
                        np.asarray(current["arms"][name]["plug"]) - baseline["arms"][name]["plug"]
                    )
                    > 0.004
                ):
                    raise RuntimeError("Terminal moved before descent; need new image")
            move("approach", grips, ready_paths=prepared["approach"])
            target_grip = {name: 0.0 for name in selected}
            closed = dwell("close", 4)[-1]
            # Start the shared translation from the measured closed poses rather
            # than independently changing the distance between held endpoints.
            lifts, paths = shared_lift(
                selected, {name: np.asarray(closed["arms"][name]["q_rad"]) for name in selected}
            )
            move("lift", lifts, ready_paths=paths)
            held = dwell("hold", 4)
            holds = {
                name: assess_hold(
                    baseline["arms"][name], [s["arms"][name] for s in held], closed["arms"][name]
                )
                for name in selected
            }
            report.update(
                holds=holds,
                baseline=baseline,
                closed=closed,
                pickup_hold_success=all(h["success"] for h in holds.values()),
            )
            report["model_sha256"] = {
                name: hashlib.sha256((args.models / (name + ".urdf")).read_bytes()).hexdigest()
                for name in selected
            }
            if args.place_in_nest:
                if not report["pickup_hold_success"]:
                    raise RuntimeError("Holding failed; refusing nest transfer")
                primary = next(
                    name
                    for name, assigned_row in assigned.items()
                    if assigned_row["plug_id"] == args.plug
                )
                transport_reference = held[-1]["arms"]
                current = {name: np.asarray(held[-1]["arms"][name]["q_rad"]) for name in selected}
                nest_plan = plan_to_nest(
                    selected,
                    collision_bounds,
                    args.world,
                    current,
                    primary,
                    assigned[primary]["grasp_candidate"]["connector_from_tool"],
                )
                report["nest_plan"] = nest_plan
                if not nest_plan["success"]:
                    raise RuntimeError("No continuous collision-screened nest transfer")
                for phase in nest_plan["attempts"][nest_plan["selected_attempt"]]["phases"]:
                    paths = {name: np.asarray(path) for name, path in phase["joint_paths"].items()}
                    targets = {
                        name: selected[name].forward(path[-1]) for name, path in paths.items()
                    }
                    move("nest_transfer_" + str(phase["index"]), targets, ready_paths=paths)
                target_grip[primary] = OPEN_TRAVEL_M
                dwell("nest_primary_open", 3)
                transport_reference = {
                    name: reference
                    for name, reference in transport_reference.items()
                    if name != primary
                }
                live = sample()
                start_q = {name: np.asarray(live["arms"][name]["q_rad"]) for name in selected}
                retreat = selected[primary].forward(start_q[primary])
                retreat[:3, 3] -= retreat[:3, 2] * 0.08
                primary_path = translation_path(selected[primary], start_q[primary], retreat)
                paths = {
                    name: primary_path
                    if name == primary
                    else np.repeat([start_q[name]], len(primary_path), axis=0)
                    for name in selected
                }
                move(
                    "nest_retreat",
                    {name: selected[name].forward(path[-1]) for name, path in paths.items()},
                    ready_paths=paths,
                )
                seated = dwell("nest_observe_seat", 2)
                goal = np.asarray(nest_plan["goal_connector_pose"])
                report["nest_seat"] = assess_seat(
                    [{**row["arms"][primary], "time_s": row["sim_time_s"]} for row in seated],
                    center=goal[:3, 3],
                    rotation=goal[:3, :3],
                )
                report.update(
                    success=report["nest_seat"]["success"],
                    stage="paired_pickup_and_nest_placement",
                    final=seated[-1],
                    scope="Fresh RGB-D paired contact pickup, continuous nest transfer and primary release/retreat; support holds another terminal. No top regrasp or board insertion",
                )
                if not report["success"]:
                    raise RuntimeError("Nest seat did not remain stable after primary retreat")
                # Supporting arm remains closed until the trial cleanup; final_open
                # is a separate teardown phase, not part of the seated window.
                return
            target_grip = {name: OPEN_TRAVEL_M for name in selected}
            released = dwell("release", 3)[-1]
            falls = {
                name: held[-1]["arms"][name]["plug"][2] - released["arms"][name]["plug"][2]
                for name in selected
            }
            report.update(
                holds=holds,
                release_drops_m=falls,
                baseline=baseline,
                closed=closed,
                final=released,
                success=all(hold["success"] and falls[name] > 0.03 for name, hold in holds.items()),
            )
            if len(selected) == 2:
                cooperation = assess_cooperation(closed, held)
                report["cooperation"] = cooperation
                report["success"] = report["success"] and cooperation["success"]
            if len(selected) == 1:
                name = next(iter(selected))
                report.update(arm=name, hold=holds[name], release_drop_m=falls[name])
        except Exception as error:
            report["error"] = str(error)
        finally:
            if target_q is not None:
                current = sample()
                if current is not None:
                    target_q = {
                        name: np.asarray(current["arms"][name]["q_rad"]) for name in selected
                    }
                target_grip = {name: OPEN_TRAVEL_M for name in selected}
                try:
                    dwell("final_open", 1)
                except RuntimeError:
                    pass
            report["inter_arm_contacts"] = contacts
            report["contact_messages"] = contact_updates[0]
            report["contact_diagnostics"] = list(contact_diagnostics.copy().values())
            report["contact_diagnostics_scope"] = (
                "Left-arm sensors only; message counts may include duplicate sensors. "
                "Penetration depth is not measured contact force."
            )
            transport.unsubscribe(CONTACT_TOPIC)
            transport.unsubscribe(pose_topic)
            if bridge:
                stop_process(bridge, group=True)
            if throttled:
                try:
                    set_camera_rate(original_rate)
                    report["camera_rate_restored_hz"] = original_rate
                except (subprocess.SubprocessError, OSError) as error:
                    report["camera_restore_error"] = str(error)
            node.destroy_node()
            rclpy.shutdown()
            (args.output / "trajectory.json").write_text(json.dumps(history, indent=2) + "\n")
            (args.output / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if not report["success"]:
        raise RuntimeError("MELFA pickup trial failed; see validation.json")


if __name__ == "__main__":
    main()
