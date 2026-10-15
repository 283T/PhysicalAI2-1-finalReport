"""Small parallel-jaw prototype with two independent prismatic finger joints."""

import math
import xml.etree.ElementTree as ET

from build_world import child, plugin

FINGER_REST_X_M = 0.012
FINGER_WIDTH_M = 0.008
MAX_TRAVEL_M = 0.020
OPEN_TRAVEL_M = 0.018


def gap_m(travel, rest_gap_m=0.016):
    return rest_gap_m + 2 * travel


def measured_rest_gap(urdf, prefix):
    """Read opposing pad inner faces at zero travel; reject unsupported geometry."""
    robot = ET.parse(urdf).getroot()
    faces = []
    for name, side in (("left", -1), ("right", 1)):
        joint = robot.find(f"joint[@name='{prefix}finger_{name}_joint']")
        finger = robot.find(f"link[@name='{prefix}finger_{name}']")
        if joint is None or finger is None or joint.get("type") != "prismatic":
            raise ValueError("Two known prismatic fingers required")
        if joint.find("axis").get("xyz") != f"{side} 0 0":
            raise ValueError("Opposed X-axis finger travel required")
        base = [float(v) for v in joint.find("origin").get("xyz").split()]
        if joint.find("origin").get("rpy", "0 0 0") != "0 0 0":
            raise ValueError("Unrotated finger origins required")
        inner = []
        for pad in finger.findall("collision"):
            if not pad.get("name", "").startswith("contact_pad"):
                continue
            origin = pad.find("origin")
            if origin.get("rpy", "0 0 0") != "0 0 0":
                raise ValueError("Unrotated contact pads required")
            xyz = [float(v) for v in origin.get("xyz").split()]
            size = [float(v) for v in pad.find("geometry/box").get("size").split()]
            if (
                len(xyz) != 3
                or len(size) != 3
                or not all(math.isfinite(v) for v in [*base, *xyz, *size])
                or min(size) <= 0
            ):
                raise ValueError("Finite positive pad geometry required")
            inner.append(base[0] + xyz[0] - side * size[0] / 2)
        if not inner or max(inner) - min(inner) > 1e-9:
            raise ValueError("Coplanar contact pad inner faces required")
        faces.append(inner[0])
    gap = faces[1] - faces[0]
    if not math.isfinite(gap) or gap <= 0:
        raise ValueError("Positive rest gap required")
    return gap


def add_controllers(model, prefix, topic_root, p_gain=1000):
    if not math.isfinite(p_gain) or p_gain <= 0:
        raise ValueError("Positive finite gripper gain required")
    for finger in ("left", "right"):
        plugin(
            model,
            ("joint-position-controller", "JointPositionController"),
            joint_name=prefix + "finger_" + finger + "_joint",
            topic=f"{topic_root}/{finger}/cmd",
            p_gain=p_gain,
            d_gain=5,
            i_gain=40,
            i_max=12,
            i_min=-12,
            cmd_max=12,
            cmd_min=-12,
        )


def add_gripper(
    robot,
    prefix,
    parent,
    *,
    split_contacts=False,
    backstop=False,
    capture_fences=False,
    front_fence_side="both",
    fence_width_m=0.012,
    housing_end_stops=False,
    pin_side_tip_recess_m=0.0,
    housing_end_clearance_m=0.0003,
    pad_inward_extension_m=0.0,
):
    if not math.isfinite(pad_inward_extension_m) or not 0 <= pad_inward_extension_m <= 0.005:
        raise ValueError("Pad extension must be finite and between 0 and 5 mm")
    if pad_inward_extension_m and (
        backstop or capture_fences or housing_end_stops or pin_side_tip_recess_m
    ):
        raise ValueError("Wire pads require the bare, full-length finger profile")
    if (
        not math.isfinite(housing_end_clearance_m)
        or not 0.00005 <= housing_end_clearance_m <= 0.001
    ):
        raise ValueError("Housing end clearance must be finite and between 0.05 and 1 mm")
    if housing_end_clearance_m != 0.0003 and not housing_end_stops:
        raise ValueError("A custom housing clearance requires end stops")
    if not math.isfinite(pin_side_tip_recess_m) or not 0 <= pin_side_tip_recess_m <= 0.003:
        raise ValueError("Pin-side tip recess must be finite and between 0 and 3 mm")
    if not math.isfinite(fence_width_m) or not 0.004 <= fence_width_m <= 0.012:
        raise ValueError("Fence width must be finite and between 4 and 12 mm")
    if housing_end_stops and not capture_fences:
        raise ValueError("Housing end stops require capture fences")
    if front_fence_side not in ("both", "left", "right"):
        raise ValueError("Unknown front fence side")
    if front_fence_side != "both" and not capture_fences:
        raise ValueError("A single front fence requires capture fences")
    tool = child(robot, "link", name=prefix + "gripper")
    inertial = child(tool, "inertial")
    child(inertial, "mass", value=".05")
    child(inertial, "inertia", ixx=".00002", iyy=".00002", izz=".00002", ixy="0", ixz="0", iyz="0")
    for kind in ("visual", "collision"):
        shape = child(tool, kind)
        child(shape, "origin", xyz="0 0 -.006", rpy="0 0 0")
        child(child(shape, "geometry"), "box", size=".070 .04 .012")
    fixed = child(robot, "joint", name=prefix + "gripper_mount", type="fixed")
    child(fixed, "parent", link=parent)
    child(fixed, "child", link=prefix + "gripper")
    for name, side in (("left", -1), ("right", 1)):
        finger = child(robot, "link", name=prefix + "finger_" + name)
        length = 0.080 - (pin_side_tip_recess_m if name == "left" else 0.0)
        inertial = child(finger, "inertial")
        has_ends = housing_end_stops and name == "right"
        child(inertial, "origin", xyz=".0004 0 .0505" if has_ends else f"0 0 {length / 2}")
        has_front = capture_fences and front_fence_side in ("both", name)
        child(
            inertial,
            "mass",
            value=".016"
            if has_ends
            else ".012"
            if has_front
            else ".011"
            if backstop or capture_fences
            else ".01",
        )
        child(
            inertial,
            "inertia",
            ixx=".000012" if has_ends else ".000006",
            iyy=".000012" if has_ends else ".000006",
            izz=".000002" if has_ends else ".000001",
            ixy="0",
            ixz="-.00000025" if has_ends else "0",
            iyz="0",
        )
        width = FINGER_WIDTH_M + pad_inward_extension_m
        pad_x = -side * pad_inward_extension_m / 2
        if pad_inward_extension_m:
            # Same assumed density as the original bare rectangular finger.
            finger_mass = 0.01 * width / FINGER_WIDTH_M
            inertial.find("origin").set("xyz", f"{pad_x} 0 {length / 2}")
            inertial.find("mass").set("value", str(finger_mass))
            inertia = inertial.find("inertia")
            for key, value in (
                ("ixx", finger_mass * (0.020**2 + length**2) / 12),
                ("iyy", finger_mass * (width**2 + length**2) / 12),
                ("izz", finger_mass * (width**2 + 0.020**2) / 12),
            ):
                inertia.set(key, str(value))
        visual = child(finger, "visual")
        child(visual, "origin", xyz=f"{pad_x} 0 {length / 2}", rpy="0 0 0")
        child(
            child(visual, "geometry"),
            "box",
            size=f"{width} {0.026 if housing_end_stops else 0.020} {length}",
        )
        # Two separated contact pads provide friction at distinct points along
        # the housing's long axis, rather than a single central contact pair.
        pad_offset = 0.009 if housing_end_stops else 0.006
        for index, y in enumerate((-pad_offset, pad_offset) if split_contacts else (0,)):
            shape = child(finger, "collision", name=f"contact_pad_{index}")
            child(shape, "origin", xyz=f"{pad_x} {y} {length / 2}", rpy="0 0 0")
            child(
                child(shape, "geometry"),
                "box",
                size=f"{width} {0.008 if split_contacts else 0.026 if housing_end_stops else 0.020} {length}",
            )
        if backstop or capture_fences:
            # A back-face fence resists yaw around the jaw closing axis. It
            # retracts with the finger. Width controls inward ledge reach; opening
            # alone does not guarantee release of a housing that follows a ledge.
            # TCP is 80 mm from the root, housing rear face is TCP-Z=-8 mm.
            # Leave 0.3 mm nominal clearance and keep the fence inside +/-13 mm
            # along tool-Y, clear of the nest's housing-end supports and corners.
            fences = (
                (("rear_fence", 0.071), ("front_fence", 0.093))
                if capture_fences
                else (("backstop", 0.0712),)
            )
            for fence_name, z in fences:
                if fence_name == "front_fence" and not has_front:
                    continue
                for kind in ("collision", "visual"):
                    fence = child(finger, kind, name=fence_name)
                    child(fence, "origin", xyz=f"{-side * fence_width_m / 2} 0 {z}", rpy="0 0 0")
                    child(child(fence, "geometry"), "box", size=f"{fence_width_m} .026 .001")
        if has_ends:
            # Wrap the housing's long-axis corners on the upper finger. Keep the
            # central cable outlet and the 16 mm end-face tag clear.
            end_y = 0.021 + housing_end_clearance_m
            pieces = [
                ("end_stop_spine", ".004 .045 .020", ".002 0 .082"),
                ("housing_end_minus", ".005 .002 .020", f"-.0025 {-end_y} .082"),
                ("housing_end_plus", ".005 .002 .020", f"-.0025 {end_y} .082"),
            ]
            for piece, size, xyz in pieces:
                for kind in ("collision", "visual"):
                    shape = child(finger, kind, name=piece)
                    child(shape, "origin", xyz=xyz, rpy="0 0 0")
                    child(child(shape, "geometry"), "box", size=size)
        joint = child(robot, "joint", name=prefix + "finger_" + name + "_joint", type="prismatic")
        child(joint, "origin", xyz=f"{side * FINGER_REST_X_M} 0 0", rpy="0 0 0")
        child(joint, "parent", link=prefix + "gripper")
        child(joint, "child", link=prefix + "finger_" + name)
        child(joint, "axis", xyz=f"{side} 0 0")
        child(joint, "limit", lower="0", upper=str(MAX_TRAVEL_M), effort="12", velocity=".03")
        child(joint, "dynamics", damping="2", friction=".1")
