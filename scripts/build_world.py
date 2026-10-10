"""Generate a self-contained Cartesian robot and two-pin insertion scene."""

from pathlib import Path
import json
import xml.etree.ElementTree as ET


def child(parent, tag, text=None, **attrs):
    element = ET.SubElement(parent, tag, attrs)
    if text is not None:
        element.text = str(text)
    return element


def plugin(parent, system, **parameters):
    element = child(parent, "plugin", filename=f"gz-sim-{system[0]}-system",
                    name=f"gz::sim::systems::{system[1]}")
    for key, value in parameters.items():
        child(element, key, value)
    return element


def box(link, name, size, pose, color, collision=True):
    for kind in (["collision", "visual"] if collision else ["visual"]):
        element = child(link, kind, name=f"{name}_{kind}")
        child(element, "pose", pose)
        geometry = child(element, "geometry")
        child(child(geometry, "box"), "size", size)
        if kind == "visual":
            material = child(element, "material")
            child(material, "diffuse", color)
            child(material, "ambient", color)


def link(model, name, gravity=True):
    element = child(model, "link", name=name)
    child(element, "gravity", str(gravity).lower())
    inertial = child(element, "inertial")
    child(inertial, "mass", 0.1)
    inertia = child(inertial, "inertia")
    for key in ("ixx", "iyy", "izz"):
        child(inertia, key, 0.0001)
    return element


def main():
    root = ET.Element("sdf", version="1.9")
    world = child(root, "world", name="pcb_insertion")
    physics = child(world, "physics", name="physics", type="ignored")
    child(physics, "max_step_size", 0.001)
    child(physics, "real_time_factor", 1)
    child(world, "gravity", "0 0 -9.81")
    for system in (("physics", "Physics"), ("user-commands", "UserCommands"),
                   ("scene-broadcaster", "SceneBroadcaster")):
        plugin(world, system)
    light = child(world, "light", name="sun", type="directional")
    child(light, "pose", "0 0 3 0 0 0")
    child(light, "diffuse", "0.9 0.9 0.9 1")
    child(light, "direction", "-0.3 -0.3 -1")

    fixtures = child(world, "model", name="fixtures")
    child(fixtures, "static", "true")
    base = link(fixtures, "fixture_link")
    box(base, "table", "1.2 0.8 0.04", "0 0 0.02 0 0 0", "0.4 0.4 0.45 1")
    box(base, "supply", "0.1 0.1 0.08", "-0.25 0 0.08 0 0 0", "0.4 0.5 0.7 1")

    # Split a board into boxes, omitting actual collision/visual cells for each hole.
    root_dir = Path(__file__).resolve().parents[1]
    board = json.loads((root_dir / "configs/board.json").read_text())
    ox, oy, top_z = board["origin_m"]
    width, height = (v / 1000 for v in board["size_mm"])
    thickness = board["thickness_mm"] / 1000
    half_hole = board["hole_width_mm"] / 2000
    half_spacing = board["pin_spacing_mm"] / 2000
    holes = []
    for socket in board["sockets"]:
        cx, cy = ox + socket["x_mm"] / 1000, oy + socket["y_mm"] / 1000
        holes.extend((cx + offset, cy) for offset in (-half_spacing, half_spacing))
    xs = sorted({ox, ox + width, *(edge for x, y in holes for edge in (x-half_hole, x+half_hole))})
    ys = sorted({oy, oy + height, *(edge for x, y in holes for edge in (y-half_hole, y+half_hole))})
    for i, (left, right) in enumerate(zip(xs, xs[1:])):
        for j, (bottom, top) in enumerate(zip(ys, ys[1:])):
            cx, cy = (left+right)/2, (bottom+top)/2
            if any(abs(cx-x) < half_hole and abs(cy-y) < half_hole for x, y in holes):
                continue
            box(base, f"pcb_{i}_{j}", f"{right-left} {top-bottom} {thickness}",
                f"{cx} {cy} {top_z-thickness/2} 0 0 0", "0.05 0.45 0.15 1")

    part = child(world, "model", name="connector")
    child(part, "pose", "-0.25 0 0.17 0 0 0")
    body = link(part, "body")
    box(body, "housing", "0.04 0.02 0.02", "0 0 0 0 0 0", "0.9 0.6 0.1 1")
    for i, x in enumerate((-0.01, 0.01)):
        for kind in ("collision", "visual"):
            pin = child(body, kind, name=f"pin_{i}_{kind}")
            child(pin, "pose", f"{x} 0 -0.03 0 0 0")
            cylinder = child(child(pin, "geometry"), "cylinder")
            child(cylinder, "radius", 0.002)
            child(cylinder, "length", 0.04)
            if kind == "visual":
                child(child(pin, "material"), "diffuse", "0.8 0.8 0.8 1")
    plugin(part, ("pose-publisher", "PosePublisher"), publish_link_pose="false",
           publish_model_pose="true", publish_nested_model_pose="true",
           use_pose_vector_msg="true", update_frequency=50, topic="/connector/pose")

    robot = child(world, "model", name="gantry")
    child(robot, "pose", "-0.25 0 0.2 0 0 0")
    child(robot, "self_collide", "true")
    base = link(robot, "base", gravity=False)
    box(base, "rail", "0.8 0.025 0.025", "0.25 0 0.25 0 0 0", "0.6 0.6 0.65 1")
    fixed = child(robot, "joint", name="world_fixed", type="fixed")
    child(fixed, "parent", "world")
    child(fixed, "child", "base")
    previous = "base"
    for axis_name, xyz, limits in (("x", "1 0 0", "-0.1 0.7"),
                                   ("y", "0 1 0", "-0.2 0.3"),
                                   ("z", "0 0 1", "-0.08 0.2")):
        name = f"{axis_name}_carriage"
        carriage = link(robot, name, gravity=False)
        if axis_name == "x":
            box(carriage, "cross_rail", "0.025 0.6 0.025", "0 0.1 0.22 0 0 0",
                "0.7 0.7 0.75 1")
        elif axis_name == "y":
            box(carriage, "vertical_rail", "0.025 0.025 0.25", "0 0.06 0.1 0 0 0",
                "0.6 0.6 0.65 1")
        else:
            # Idealized gripper: fingers are visual-only; rigid attachment holds the part.
            box(carriage, "tool", "0.065 0.05 0.025", "0 0 0.02 0 0 0",
                "0.1 0.2 0.6 1", collision=False)
            for i, x in enumerate((-0.026, 0.026)):
                box(carriage, f"finger_{i}", "0.008 0.025 0.045",
                    f"{x} 0 -0.015 0 0 0", "0.25 0.25 0.3 1", collision=False)
        joint = child(robot, "joint", name=f"{axis_name}_joint", type="prismatic")
        child(joint, "parent", previous)
        child(joint, "child", name)
        axis = child(joint, "axis")
        child(axis, "xyz", xyz)
        limit = child(axis, "limit")
        lo, hi = limits.split()
        child(limit, "lower", lo)
        child(limit, "upper", hi)
        child(limit, "effort", 100)
        child(limit, "velocity", 0.15)
        plugin(robot, ("joint-position-controller", "JointPositionController"),
               joint_name=f"{axis_name}_joint", topic=f"/gantry/{axis_name}/cmd",
               p_gain=4000, i_gain=0, d_gain=100, cmd_max=100, cmd_min=-100)
        previous = name
    plugin(robot, ("joint-state-publisher", "JointStatePublisher"), topic="/gantry/joints")
    plugin(robot, ("detachable-joint", "DetachableJoint"), parent_link="z_carriage",
           child_model="connector", child_link="body", attach_topic="/grasp/attach",
           detach_topic="/grasp/detach", output_topic="/grasp/state")
    ET.indent(root)
    target = Path(__file__).resolve().parents[1] / "sim/worlds/insertion.sdf"
    ET.ElementTree(root).write(target, encoding="utf-8", xml_declaration=True)
    print(target)


if __name__ == "__main__":
    main()
