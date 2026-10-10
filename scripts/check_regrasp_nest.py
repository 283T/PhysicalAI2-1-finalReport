"""Physical passive-nest test with an unanchored pin-equipped flexible harness."""

import argparse
import hashlib
import json
import os
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from arm_kinematics import quaternion_rotation, rotation_error
from build_case import harness_model
from build_world import box, child, link, plugin
from grasp_validation import world_link_positions
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
from processes import stop_process
from regrasp_fixture import add_regrasp_nest


def build(output, with_nest):
    root = ET.Element("sdf", version="1.10")
    world = child(root, "world", name="regrasp_nest_test")
    child(world, "gravity", "0 0 -9.81")
    physics = child(world, "physics", name="physics", type="ignored")
    child(physics, "max_step_size", 0.001)
    for system in (("physics", "Physics"), ("scene-broadcaster", "SceneBroadcaster")):
        plugin(world, system)
    fixtures = child(world, "model", name="fixtures")
    child(fixtures, "static", "true")
    base = link(fixtures, "fixture_link")
    box(base, "table", "1 .8 .04", "0 0 .02 0 0 0", ".3 .3 .3 1")
    if with_nest:
        add_regrasp_nest(base)
    harness_model(
        world,
        {
            "pose_m_rad": [-0.32, 0, 0.235, 0, 0, 0],
            "branches": [
                {
                    "plug_id": "P1",
                    "segments": 12,
                    "angle_rad": 0,
                    "curve_rad": 0,
                    "color": ".9 .25 .08 1",
                }
            ],
        },
    )
    ET.indent(root)
    output.mkdir(parents=True, exist_ok=True)
    path = output / "world.sdf"
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    return path


def trial(output, with_nest):
    scene = build(output, with_nest)
    os.environ["GZ_PARTITION"] = f"regrasp-nest-{os.getpid()}-{int(with_nest)}"
    transport = Node()
    samples = []

    def receive(msg):
        poses = {
            p.name: {
                "xyz": [p.position.x, p.position.y, p.position.z],
                "q": [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w],
            }
            for p in msg.pose
        }
        if not all(name in poses for name in ("wire_harness_multi", "P1", "junction")):
            return
        links = [name for name in poses if name.startswith("P1_wire_")] + ["P1", "junction"]
        points = world_link_positions(poses, "wire_harness_multi", links)
        rotation = quaternion_rotation(poses["wire_harness_multi"]["q"]) @ quaternion_rotation(
            poses["P1"]["q"]
        )
        samples.append(
            {
                "sim_time_s": msg.header.stamp.sec + msg.header.stamp.nsec / 1e9,
                "plug_xyz_m": points["P1"].tolist(),
                "angle_error_rad": float(np.linalg.norm(rotation_error(np.eye(3), rotation))),
                "cable_z_span_m": float(np.ptp([p[2] for p in points.values()])),
            }
        )

    topic = "/world/regrasp_nest_test/dynamic_pose/info"
    transport.subscribe(Pose_V, topic, receive)
    report = {
        "success": False,
        "with_nest": with_nest,
        "scope": "Single pin-equipped terminal with a 12-link flexible tail, released initial condition; no robot, perception, regrasp transition, dual-arm support or insertion",
    }
    report["world_sha256"] = hashlib.sha256(scene.read_bytes()).hexdigest()
    process = None
    try:
        with (output / "simulation.log").open("w") as log:
            process = subprocess.Popen(
                ["gz", "sim", "-s", "-r", str(scene)],
                stdout=log,
                stderr=log,
                start_new_session=True,
            )
            deadline = time.monotonic() + 60
            while not samples or samples[-1]["sim_time_s"] < 6:
                if time.monotonic() > deadline:
                    raise RuntimeError("No six-second physics observation")
                time.sleep(0.05)
            settled = [row for row in samples if 3 <= row["sim_time_s"] <= 6]
            if len(settled) < 30:
                raise RuntimeError("Insufficient fresh settled observations")
            points = np.array([row["plug_xyz_m"] for row in settled])
            lateral = float(np.max(np.linalg.norm(points[:, :2], axis=1)))
            height_error = float(np.max(np.abs(points[:, 2] - 0.23)))
            angle = float(max(row["angle_error_rad"] for row in settled))
            drop = float(0.235 - np.max(points[:, 2]))
            deformation = float(min(row["cable_z_span_m"] for row in settled))
            report.update(
                success=bool(
                    lateral < 0.003 and height_error < 0.002 and angle < 0.12 and deformation > 0.05
                )
                if with_nest
                else drop > 0.05,
                maximum_lateral_offset_m=lateral,
                maximum_seat_height_error_m=height_error,
                maximum_angle_error_rad=angle,
                minimum_drop_m=drop,
                minimum_cable_z_span_m=deformation,
                settled_samples=len(settled),
            )
    except Exception as error:
        report["error"] = str(error)
        raise
    finally:
        transport.unsubscribe(topic)
        if process is not None:
            stop_process(process, group=True)
        (output / "samples.json").write_text(json.dumps(samples, indent=2) + "\n")
        (output / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "validation.json").unlink(missing_ok=True)
    results = [
        trial(args.output / name, enabled)
        for name, enabled in (("with_nest", True), ("without_nest", False))
    ]
    report = {"success": all(row["success"] for row in results), "trials": results}
    (args.output / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if not report["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
