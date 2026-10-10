"""Generate a self-contained Cartesian robot and two-pin insertion scene."""

from pathlib import Path
import json
import math
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


def add_board(base, board):
    """Build receptacles and matching physical board holes from one design."""
    ox, oy, top_z = board["origin_m"]
    panel_top = top_z - board["receptacle_depth_mm"] / 1000
    width, height = (v / 1000 for v in board["size_mm"])
    thickness = board["thickness_mm"] / 1000
    half_hole = board["hole_width_mm"] / 2000
    half_spacing = board["pin_spacing_mm"] / 2000
    holes = []
    for socket in board["sockets"]:
        cx, cy = ox + socket["x_mm"] / 1000, oy + socket["y_mm"] / 1000
        angle = math.radians(socket['yaw_deg'])
        holes.extend((cx + offset*math.cos(angle), cy + offset*math.sin(angle))
                     for offset in (-half_spacing, half_spacing))
    xs = sorted({ox, ox + width, *(edge for x, y in holes for edge in (x-half_hole, x+half_hole))})
    ys = sorted({oy, oy + height, *(edge for x, y in holes for edge in (y-half_hole, y+half_hole))})
    for i, (left, right) in enumerate(zip(xs, xs[1:])):
        for j, (bottom, top) in enumerate(zip(ys, ys[1:])):
            cx, cy = (left+right)/2, (bottom+top)/2
            if any(abs(cx-x) < half_hole and abs(cy-y) < half_hole for x, y in holes):
                continue
            box(base, f"pcb_{i}_{j}", f"{right-left} {top-bottom} {thickness}",
                f"{cx} {cy} {panel_top-thickness/2} 0 0 0", "0.05 0.45 0.15 1")

    # Raised female receptacles with actual empty channels; the plug seats on their rim.
    depth = board["receptacle_depth_mm"] / 1000
    for socket in board["sockets"]:
        cx, cy = ox + socket["x_mm"]/1000, oy + socket["y_mm"]/1000
        angle = math.radians(socket['yaw_deg'])
        rx = [cx-.03, cx-.01-half_hole, cx-.01+half_hole,
              cx+.01-half_hole, cx+.01+half_hole, cx+.03]
        ry = [cy-.02, cy-half_hole, cy+half_hole, cy+.02]
        for i, (left, right) in enumerate(zip(rx, rx[1:])):
            for j, (bottom, top) in enumerate(zip(ry, ry[1:])):
                if i in (1, 3) and j == 1:
                    continue
                dx, dy = (left+right)/2-cx, (bottom+top)/2-cy
                sx = cx + dx*math.cos(angle)-dy*math.sin(angle)
                sy = cy + dx*math.sin(angle)+dy*math.cos(angle)
                box(base, f"socket_{socket['id']}_{i}_{j}", f"{right-left} {top-bottom} {depth}",
                    f"{sx} {sy} {top_z-depth/2} 0 0 {angle}", socket.get("color", "0.15 0.3 0.5 1"))



def main():
    root = ET.Element("sdf", version="1.9")
    world = child(root, "world", name="harness_connection")
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
    add_board(base, board)

    part = child(world, "model", name="wire_harness")
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
    # Two short leads are rigid visual geometry. Cable bending, forces and snagging are not modeled.
    for wire, x, color in ((0, -.007, "0.8 0.08 0.04 1"), (1, .007, "0.08 0.08 0.1 1")):
        points = [(x, -.01, .007), (x, -.035, .018), (x+.012, -.07, .025),
                  (x+.028, -.105, .018), (x+.028, -.14, .002)]
        for index, (start, end) in enumerate(zip(points, points[1:])):
            delta = [b-a for a, b in zip(start, end)]
            length = math.sqrt(sum(v*v for v in delta))
            center = [(a+b)/2 for a, b in zip(start, end)]
            yaw = math.atan2(delta[1], delta[0])
            pitch = math.acos(delta[2]/length)
            visual = child(body, "visual", name=f"wire_{wire}_{index}")
            child(visual, "pose", " ".join(str(v) for v in (*center, 0, pitch, yaw)))
            cylinder = child(child(visual, "geometry"), "cylinder")
            child(cylinder, "radius", .003)
            child(cylinder, "length", length)
            material = child(visual, "material")
            child(material, "diffuse", color)
            child(material, "ambient", color)

    plugin(part, ("pose-publisher", "PosePublisher"), publish_link_pose="false",
           publish_model_pose="true", publish_nested_model_pose="true",
           use_pose_vector_msg="true", update_frequency=50, topic="/harness/pose")

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
           child_model="wire_harness", child_link="body", attach_topic="/grasp/attach",
           detach_topic="/grasp/detach", output_topic="/grasp/state")
    ET.indent(root)
    target = Path(__file__).resolve().parents[1] / "sim/worlds/insertion.sdf"
    ET.ElementTree(root).write(target, encoding="utf-8", xml_declaration=True)
    print(target)


if __name__ == "__main__":
    main()
