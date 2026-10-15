"""Factory prototype: articulated arms, conveyor carrier and a jointed flexible cable."""

import argparse
import copy
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from build_world import box, child, link, plugin
from gripper_model import add_controllers, add_gripper
from project_paths import ROOT


def mass(element, value):
    element.find("inertial/mass").text = str(value)
    for item in element.findall("inertial/inertia/*"):
        item.text = str(value * 0.025**2 / 12)


def flexible_cable(model, count=12):
    body = model.find("link[@name='body']")
    for item in list(body):
        if item.tag == "visual" and item.get("name", "").startswith("wire_"):
            body.remove(item)
    previous = "body"
    length = 0.025
    for index in range(count):
        name = f"cable_{index}"
        segment = link(model, name)
        mass(segment, 0.004)
        # Link frames are at segment centers; chain extends from the plug along -Y.
        child(segment, "pose", f"0 {-0.015 - (index + 0.5) * length} .008 0 0 0")
        box(
            segment,
            "insulation",
            f".008 {length} .008",
            "0 0 0 0 0 0",
            ".8 .08 .04 1" if index % 2 == 0 else ".08 .08 .1 1",
        )
        joint = child(model, "joint", name=f"bend_{index}", type="universal")
        child(joint, "parent", previous)
        child(joint, "child", name)
        child(joint, "pose", f"0 {length / 2} 0 0 0 0")
        for tag, direction in (("axis", "1 0 0"), ("axis2", "0 0 1")):
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
        previous = name
    # Report link poses for deformation verification; keep separate from plug-only pose topic.
    plugin(
        model,
        ("pose-publisher", "PosePublisher"),
        publish_link_pose="true",
        publish_model_pose="false",
        use_pose_vector_msg="true",
        update_frequency=30,
        topic="/cable/poses",
    )


JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def joint_names(robot_type):
    return (
        tuple(f"rv4frl_joint_{i}" for i in range(1, 7)) if robot_type == "melfa_rv4frl" else JOINTS
    )


def arm(
    world,
    name,
    origin,
    yaw,
    robot_type="ur3e",
    urdf_output=None,
    *,
    split_contacts=False,
    gripper_p_gain=1000,
    backstop=False,
    capture_fences=False,
    front_fence_side="both",
    fence_width_m=0.012,
    housing_end_stops=False,
    pin_side_tip_recess_m=0.0,
    housing_end_clearance_m=0.0003,
    pad_inward_extension_m=0.0,
    wire_contact_length_m=0.020,
):
    """Load official robot geometry and model parameters; MELFA masses are placeholders."""
    import subprocess
    import tempfile

    from ament_index_python.packages import get_package_share_directory

    prefix = name + "_"
    if robot_type == "melfa_rv4frl":
        description = Path(get_package_share_directory("melfa_description"))
        urdf = subprocess.check_output(
            ["xacro", str(ROOT / "sim/robots/melfa_rv4frl.xacro"), f"prefix:={prefix}"], text=True
        )
        urdf = urdf.replace("package://melfa_description/", description.as_uri() + "/")
        tool_parent = prefix + "rv4frl_default_tcp"
    else:
        description = Path(get_package_share_directory("ur_description"))
        urdf = subprocess.check_output(
            [
                "xacro",
                str(description / "urdf/ur.urdf.xacro"),
                "ur_type:=ur3e",
                f"name:={name}",
                f"tf_prefix:={prefix}",
                "force_abs_paths:=true",
            ],
            text=True,
        )
        tool_parent = prefix + "tool0"
    robot = ET.fromstring(urdf)
    add_gripper(
        robot,
        prefix,
        tool_parent,
        split_contacts=split_contacts,
        backstop=backstop,
        capture_fences=capture_fences,
        front_fence_side=front_fence_side,
        fence_width_m=fence_width_m,
        housing_end_stops=housing_end_stops,
        pin_side_tip_recess_m=pin_side_tip_recess_m,
        housing_end_clearance_m=housing_end_clearance_m,
        pad_inward_extension_m=pad_inward_extension_m,
        wire_contact_length_m=wire_contact_length_m,
    )
    if urdf_output is not None:
        ET.indent(robot)
        ET.ElementTree(robot).write(urdf_output, encoding="utf-8", xml_declaration=True)
    with tempfile.TemporaryDirectory() as directory:
        source = Path(directory) / "arm.urdf"
        ET.ElementTree(robot).write(source, encoding="utf-8", xml_declaration=True)
        converted = subprocess.check_output(["gz", "sdf", "-p", str(source)], text=True)
    model = ET.fromstring(converted).find("model")
    if model is None:
        raise RuntimeError("Robot conversion produced no model")
    model.set("name", name)
    child(model, "pose", f"{origin[0]} {origin[1]} .12 0 0 {yaw}")
    for index, joint in enumerate(joint_names(robot_type)):
        effort = float(model.find(f"joint[@name='{prefix + joint}']/axis/limit/effort").text)
        melfa = robot_type == "melfa_rv4frl"
        plugin(
            model,
            ("joint-position-controller", "JointPositionController"),
            joint_name=prefix + joint,
            topic=f"/{name}/joint_{index}/cmd",
            p_gain=(180 if index < 3 else 80) if melfa else (800 if index < 3 else 80),
            d_gain=(2 if index < 3 else 0.1) if melfa else (8 if index < 3 else 0.1),
            i_gain=(100 if index < 3 else 40) if melfa else (100 if index < 3 else 20),
            i_max=effort,
            i_min=-effort,
            cmd_max=effort,
            cmd_min=-effort,
        )
    add_controllers(model, prefix, f"/{name}/gripper", p_gain=gripper_p_gain)
    plugin(model, ("joint-state-publisher", "JointStatePublisher"), topic=f"/{name}/joints")
    world.append(model)


def main():
    from arm_contacts import add_arm_contact_sensors

    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", choices=["ur3e", "melfa_rv4frl"], default="ur3e")
    parser.add_argument("--output", type=Path, default=ROOT / "sim/worlds")
    parser.add_argument("--conveyor-p-gain", type=float, default=400)
    parser.add_argument("--wire-contact-length-mm", type=float, default=20)
    parser.add_argument("--gripper-p-gain", type=float, default=1000)
    parser.add_argument("--left-gripper-p-gain", type=float)
    parser.add_argument("--backstop-gripper", action="store_true")
    parser.add_argument("--capture-gripper", action="store_true")
    parser.add_argument("--pin-side-tip-recess-mm", type=float, default=0)
    parser.add_argument("--housing-end-clearance-mm", type=float, default=0.3)
    parser.add_argument("--housing-end-stops", action="store_true")
    parser.add_argument(
        "--left-wire-gripper",
        action="store_true",
        help="Bare left-arm pads extended inward 5 mm per finger",
    )
    parser.add_argument("--fence-width-mm", type=float, default=12)
    parser.add_argument("--front-fence-side", choices=("both", "left", "right"), default="both")
    parser.add_argument(
        "--split-finger-contacts",
        action="store_true",
        help="Use two separated contact pads per finger",
    )
    args = parser.parse_args()
    if not math.isfinite(args.conveyor_p_gain) or args.conveyor_p_gain <= 0:
        parser.error("--conveyor-p-gain must be positive and finite")
    if args.left_gripper_p_gain is not None and (
        not math.isfinite(args.left_gripper_p_gain) or args.left_gripper_p_gain <= 0
    ):
        parser.error("--left-gripper-p-gain must be positive and finite")
    if (
        not math.isfinite(args.wire_contact_length_mm)
        or not 20 <= args.wire_contact_length_mm <= 60
    ):
        parser.error("--wire-contact-length-mm must be between 20 and 60")
    if args.wire_contact_length_mm != 20 and not args.left_wire_gripper:
        parser.error("--wire-contact-length-mm requires --left-wire-gripper")
    if not math.isfinite(args.fence_width_mm) or not 4 <= args.fence_width_mm <= 12:
        parser.error("--fence-width-mm must be finite and between 4 and 12")
    if args.fence_width_mm != 12 and not (args.capture_gripper or args.backstop_gripper):
        parser.error("--fence-width-mm requires capture or backstop gripper")
    if not math.isfinite(args.pin_side_tip_recess_mm) or not 0 <= args.pin_side_tip_recess_mm <= 3:
        parser.error("--pin-side-tip-recess-mm must be finite and between 0 and 3")
    if (
        not math.isfinite(args.housing_end_clearance_mm)
        or not 0.05 <= args.housing_end_clearance_mm <= 1
    ):
        parser.error("--housing-end-clearance-mm must be finite and between 0.05 and 1")
    if args.housing_end_clearance_mm != 0.3 and not args.housing_end_stops:
        parser.error("--housing-end-clearance-mm requires --housing-end-stops")
    if args.housing_end_stops and not args.capture_gripper:
        parser.error("--housing-end-stops requires --capture-gripper")
    if args.capture_gripper and args.backstop_gripper:
        parser.error("Choose one experimental finger profile")
    if args.front_fence_side != "both" and not args.capture_gripper:
        parser.error("--front-fence-side requires --capture-gripper")
    if not math.isfinite(args.gripper_p_gain) or args.gripper_p_gain <= 0:
        parser.error("Positive finite gripper gain required")
    args.output.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(ROOT / "sim/worlds/insertion.sdf")
    world = tree.getroot().find("world")
    world.set("name", "harness_factory_prototype")
    world.remove(world.find("model[@name='gantry']"))
    fixture = world.find("model[@name='fixtures']/link")
    for item in list(fixture):
        if item.get("name", "").startswith("supply_"):
            fixture.remove(item)
    harness = world.find("model[@name='wire_harness']")
    flexible_cable(harness)
    arm(
        world,
        "left_arm",
        (-0.15, -0.28),
        math.pi,
        args.robot,
        args.output / "left_arm.urdf",
        split_contacts=args.split_finger_contacts,
        gripper_p_gain=args.left_gripper_p_gain
        if args.left_gripper_p_gain is not None
        else args.gripper_p_gain,
        backstop=args.backstop_gripper and not args.left_wire_gripper,
        capture_fences=args.capture_gripper and not args.left_wire_gripper,
        front_fence_side="both" if args.left_wire_gripper else args.front_fence_side,
        fence_width_m=args.fence_width_mm / 1000,
        housing_end_stops=args.housing_end_stops and not args.left_wire_gripper,
        pin_side_tip_recess_m=0 if args.left_wire_gripper else args.pin_side_tip_recess_mm / 1000,
        housing_end_clearance_m=0.0003
        if args.left_wire_gripper
        else args.housing_end_clearance_mm / 1000,
        pad_inward_extension_m=0.005 if args.left_wire_gripper else 0,
        wire_contact_length_m=args.wire_contact_length_mm / 1000,
    )
    arm(
        world,
        "right_arm",
        (0.38, -0.28),
        0,
        args.robot,
        args.output / "right_arm.urdf",
        split_contacts=args.split_finger_contacts,
        gripper_p_gain=args.gripper_p_gain,
        backstop=args.backstop_gripper,
        capture_fences=args.capture_gripper,
        front_fence_side=args.front_fence_side,
        fence_width_m=args.fence_width_mm / 1000,
        housing_end_stops=args.housing_end_stops,
        pin_side_tip_recess_m=args.pin_side_tip_recess_mm / 1000,
        housing_end_clearance_m=args.housing_end_clearance_mm / 1000,
    )
    # A moving carrier approximates a conveyor pallet. Transport control is a separate stage.
    carrier = child(world, "model", name="conveyor_carrier")
    base = link(carrier, "carrier", gravity=False)
    box(base, "pallet", ".12 .25 .02", "-.25 -.08 .11 0 0 0", ".35 .4 .45 1")
    joint = child(carrier, "joint", name="travel", type="prismatic")
    child(joint, "parent", "world")
    child(joint, "child", "carrier")
    axis = child(joint, "axis")
    child(axis, "xyz", "1 0 0")
    lim = child(axis, "limit")
    child(lim, "lower", -0.15)
    child(lim, "upper", 0.3)
    plugin(
        carrier,
        ("joint-position-controller", "JointPositionController"),
        joint_name="travel",
        topic="/conveyor/cmd",
        p_gain=args.conveyor_p_gain,
        d_gain=2 * math.sqrt(args.conveyor_p_gain),
        cmd_max=20,
        cmd_min=-20,
    )
    # Camera image is intended to feed perception, not simulator true object coordinates.
    camera = child(world, "model", name="overhead_camera")
    child(camera, "static", "true")
    camera_link = link(camera, "camera_link")
    child(camera_link, "pose", "0 0 .95 0 1.57079632679 0")
    sensor = child(camera_link, "sensor", name="rgb", type="camera")
    child(sensor, "always_on", "true")
    child(sensor, "update_rate", 10)
    child(sensor, "topic", "/factory/image")
    cam = child(sensor, "camera")
    child(cam, "horizontal_fov", 1.4)
    im = child(cam, "image")
    child(im, "width", 640)
    child(im, "height", 480)
    child(im, "format", "R8G8B8")
    clip = child(cam, "clip")
    child(clip, "near", 0.05)
    child(clip, "far", 3)
    plugin(world, ("sensors", "Sensors"), render_engine="ogre")
    add_arm_contact_sensors(world)
    ET.indent(tree)
    tree.write(args.output / "factory_prototype.sdf", encoding="utf-8", xml_declaration=True)
    # Keep motor validation independent of cable contact and camera rendering.
    arm_test = ET.Element("sdf", version="1.9")
    arm_world = child(arm_test, "world", name="dual_arm_validation")
    arm_physics = child(arm_world, "physics", name="physics", type="ignored")
    child(arm_physics, "max_step_size", 0.001)
    child(arm_world, "gravity", "0 0 -9.81")
    plugin(arm_world, ("physics", "Physics"))
    plugin(arm_world, ("scene-broadcaster", "SceneBroadcaster"))
    for name in ("left_arm", "right_arm"):
        arm_world.append(copy.deepcopy(world.find(f"model[@name='{name}']")))
    ET.indent(arm_test)
    ET.ElementTree(arm_test).write(
        args.output / "arms_test.sdf", encoding="utf-8", xml_declaration=True
    )
    # Independent cable test: anchor plug and let the cable droop freely beyond the table edge.
    test = ET.Element("sdf", version="1.9")
    scene = child(test, "world", name="cable_validation")
    physics = child(scene, "physics", name="physics", type="ignored")
    child(physics, "max_step_size", 0.001)
    child(scene, "gravity", "0 0 -9.81")
    plugin(scene, ("physics", "Physics"))
    plugin(scene, ("scene-broadcaster", "SceneBroadcaster"))
    cable = copy.deepcopy(harness)
    cable.find("pose").text = "0 0 .45 0 0 0"
    anchor = child(cable, "joint", name="test_anchor", type="fixed")
    child(anchor, "parent", "world")
    child(anchor, "child", "body")
    scene.append(cable)
    ET.indent(test)
    ET.ElementTree(test).write(
        args.output / "cable_test.sdf", encoding="utf-8", xml_declaration=True
    )


if __name__ == "__main__":
    main()
