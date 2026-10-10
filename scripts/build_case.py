"""Generate a matched drawing/board and one connected, randomized three-plug harness."""

import argparse
import hashlib
import json
import math
import secrets
import xml.etree.ElementTree as ET
from pathlib import Path

from assembly_case import make_case
from build_factory import mass
from build_world import add_board, box, child, link, plugin
from project_paths import ROOT


def harness_model(world, trial):
    model = child(world, "model", name="wire_harness_multi")
    child(model, "pose", " ".join(map(str, trial["pose_m_rad"])))
    hub = link(model, "junction")
    mass(hub, 0.02)
    box(hub, "bundle", ".014 .014 .014", "0 0 0 0 0 0", ".08 .08 .08 1")
    for branch in trial["branches"]:
        plug = branch["plug_id"]
        previous = "junction"
        x, y = 0.0, 0.0
        for index in range(branch["segments"]):
            angle = branch["angle_rad"] + branch["curve_rad"] * (
                index / (branch["segments"] - 1) - 0.5
            )
            dx, dy = 0.025 * math.cos(angle), 0.025 * math.sin(angle)
            name = f"{plug}_wire_{index}"
            segment = link(model, name)
            mass(segment, 0.004)
            child(segment, "pose", f"{x + dx / 2} {y + dy / 2} 0 0 0 {angle}")
            box(segment, "insulation", ".025 .008 .008", "0 0 0 0 0 0", branch["color"])
            joint = child(model, "joint", name=f"{plug}_bend_{index}", type="universal")
            child(joint, "parent", previous)
            child(joint, "child", name)
            child(joint, "pose", "-.0125 0 0 0 0 0")
            for tag, direction in (("axis", "0 1 0"), ("axis2", "0 0 1")):
                axis = child(joint, tag)
                child(axis, "xyz", direction)
                dynamics = child(axis, "dynamics")
                child(dynamics, "damping", 0.002)
                child(dynamics, "spring_reference", 0)
                child(dynamics, "spring_stiffness", 0.003)
                limits = child(axis, "limit")
                child(limits, "lower", -1.2)
                child(limits, "upper", 1.2)
                child(limits, "effort", 1)
            x, y = x + dx, y + dy
            previous = name
        terminal = link(model, plug)
        mass(terminal, 0.025)
        child(
            terminal,
            "pose",
            f"{x + 0.02 * math.cos(angle)} {y + 0.02 * math.sin(angle)} 0 0 0 {angle}",
        )
        box(terminal, "housing", ".04 .02 .02", "0 0 0 0 0 0", branch["color"])
        # Redundant positive-X marks remain visible when the housing rolls onto its side.
        for side in (-1, 1):
            box(
                terminal,
                f"orientation_mark_z_{side}",
                ".012 .006 .001",
                f".01 0 {side * 0.0105} 0 0 0",
                "1 1 1 1",
                collision=False,
            )
            box(
                terminal,
                f"orientation_mark_y_{side}",
                ".012 .001 .006",
                f".01 {side * 0.0105} 0 0 0 0",
                "1 1 1 1",
                collision=False,
            )
        for index, offset in enumerate((-0.01, 0.01)):
            for kind in ("collision", "visual"):
                pin = child(terminal, kind, name=f"pin_{index}_{kind}")
                child(pin, "pose", f"{offset} 0 -.03 0 0 0")
                cylinder = child(child(pin, "geometry"), "cylinder")
                child(cylinder, "radius", 0.002)
                child(cylinder, "length", 0.04)
        fixed = child(model, "joint", name=f"{plug}_termination", type="fixed")
        child(fixed, "parent", previous)
        child(fixed, "child", plug)
        child(fixed, "pose", "-.02 0 0 0 0 0")
    plugin(
        model,
        ("pose-publisher", "PosePublisher"),
        publish_link_pose="true",
        publish_model_pose="false",
        use_pose_vector_msg="true",
        update_frequency=30,
        topic="/multi_harness/poses",
    )
    return model


def draw(design, output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig, ax = plt.subplots(figsize=(10, 8), dpi=160)
    width, height = design["size_mm"]
    ax.add_patch(Rectangle((0, 0), width, height, facecolor="#e8eef5", edgecolor="#193e2a", lw=2))
    by_socket = {row["socket"]: row["plug_id"] for row in design["connections"]}
    for socket in design["sockets"]:
        x, y, angle = socket["x_mm"], socket["y_mm"], math.radians(socket["yaw_deg"])
        color = tuple(float(value) for value in socket["color"].split())
        ax.add_patch(
            Rectangle(
                (x - 30, y - 20),
                60,
                40,
                angle=socket["yaw_deg"],
                rotation_point="center",
                facecolor=color,
                edgecolor="black",
                alpha=0.7,
            )
        )
        for sign in (-1, 1):
            ax.add_patch(
                Rectangle(
                    (x + sign * 10 * math.cos(angle) - 4, y + sign * 10 * math.sin(angle) - 4),
                    8,
                    8,
                    facecolor="white",
                    edgecolor="black",
                )
            )
        ax.text(x, y + 25, f"{by_socket[socket['id']]} -> {socket['id']}", ha="center", fontsize=12)
        ax.text(x, y - 30, f"({x}, {y}) / {socket['yaw_deg']} deg", ha="center", fontsize=10)
    ax.text(0, height + 30, "HARNESS CONNECTION DRAWING", fontsize=17, weight="bold")
    ax.text(
        0,
        height + 17,
        f"DRAWING: {design['drawing_id']} | BOARD: {design['board_id']} | HARNESS: WH1",
        fontsize=11,
    )
    ax.text(0, -12, "ORIGIN (0, 0) | +X right, +Y up | Units: mm | NOT TO SCALE", fontsize=10)
    rows = []
    sockets = {socket["id"]: socket for socket in design["sockets"]}
    for connection in design["connections"]:
        socket = sockets[connection["socket"]]
        rows.append(
            f"PLUG: {connection['plug_id']} | SOCKET: {socket['id']} | "
            f"X={socket['x_mm']} mm, Y={socket['y_mm']} mm, YAW={socket['yaw_deg']} deg"
        )
    ax.text(
        0,
        -25,
        "\n".join(rows) + f"\nALL 3 PLUGS MUST BE CONNECTED | PANEL: {width} x {height} mm",
        fontsize=10,
        va="top",
        linespacing=1.6,
    )
    ax.set_xlim(-15, width + 15)
    ax.set_ylim(-75, height + 45)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.tight_layout()
    fig.savefig(output / "drawing.png", facecolor="white")
    plt.close(fig)


def build(case, output, factory_world=ROOT / "sim/worlds/factory_prototype.sdf"):
    output.mkdir(parents=True, exist_ok=True)
    design = case["design"]
    tree = ET.parse(factory_world)
    world = tree.getroot().find("world")
    world.set("name", "multi_connector_factory")
    # Place bases beside the pallet; use the same layout for generated scenes and IK.
    layout = json.loads((ROOT / "configs/arm_layout.json").read_text())
    for name in ("left_arm", "right_arm"):
        pose = world.find(f"model[@name='{name}']/pose")
        values = pose.text.split()
        values[0] = str(layout[name]["x_m"])
        values[1] = str(layout[name]["y_m"])
        pose.text = " ".join(values)
    world.remove(world.find("model[@name='wire_harness']"))
    fixture = world.find("model[@name='fixtures']/link")
    for item in list(fixture):
        if item.get("name", "").startswith(("pcb_", "socket_")):
            fixture.remove(item)
    add_board(fixture, design)
    for kind in ("collision", "visual"):
        fixture.find(f"{kind}[@name='table_{kind}']/geometry/box/size").text = "1.2 1.0 .04"
    pallet = world.find("model[@name='conveyor_carrier']/link")
    for shape in ("collision", "visual"):
        pallet.find(f"{shape}/geometry/box/size").text = ".60 .65 .02"
        pallet.find(f"{shape}/pose").text = "-.30 .05 .11 0 0 0"
    harness_model(world, case["simulation_initialization"])
    calibration = json.loads((ROOT / "configs/infeed_camera.json").read_text())
    camera = world.find("model[@name='overhead_camera']/link")
    camera.find("pose").text = " ".join(
        map(str, calibration["position_m"] + calibration["pose_rpy_rad"])
    )
    sensor = camera.find("sensor")
    sensor.set("type", "rgbd_camera")
    sensor.find("topic").text = calibration["topic"]
    sensor.find("update_rate").text = "5"
    sensor.find("camera/horizontal_fov").text = str(calibration["horizontal_fov_rad"])
    world.find("plugin[@name='gz::sim::systems::Sensors']/render_engine").text = "ogre2"
    ET.indent(tree)
    tree.write(output / "world.sdf", encoding="utf-8", xml_declaration=True)
    (output / "case.json").write_text(json.dumps(case, indent=2) + "\n")
    # Simulator initialization is deliberately separate from the model-facing drawing.
    (output / "board.json").write_text(json.dumps(design, indent=2) + "\n")
    draw(design, output)
    manifest = {
        name: hashlib.sha256((output / name).read_bytes()).hexdigest()
        for name in ("case.json", "board.json", "drawing.png", "world.sdf")
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--design-seed", type=int, default=secrets.randbelow(2**31))
    parser.add_argument("--placement-seed", type=int, default=secrets.randbelow(2**31))
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/multi_connector")
    parser.add_argument(
        "--factory-world", type=Path, default=ROOT / "sim/worlds/factory_prototype.sdf"
    )
    args = parser.parse_args()
    print(build(make_case(args.design_seed, args.placement_seed), args.output, args.factory_world))


if __name__ == "__main__":
    main()
