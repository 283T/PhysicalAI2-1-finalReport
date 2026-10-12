"""Small parallel-jaw prototype with two independent prismatic finger joints."""

import math

from build_world import child, plugin

FINGER_REST_X_M = 0.012
FINGER_WIDTH_M = 0.008
MAX_TRAVEL_M = 0.020
OPEN_TRAVEL_M = 0.018


def gap_m(travel):
    return 2 * (FINGER_REST_X_M + travel) - FINGER_WIDTH_M


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
    robot, prefix, parent, *, split_contacts=False, backstop=False, capture_fences=False
):
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
        inertial = child(finger, "inertial")
        child(inertial, "origin", xyz="0 0 .04")
        child(inertial, "mass", value=".012" if capture_fences else ".011" if backstop else ".01")
        child(
            inertial,
            "inertia",
            ixx=".000006",
            iyy=".000006",
            izz=".000001",
            ixy="0",
            ixz="0",
            iyz="0",
        )
        visual = child(finger, "visual")
        child(visual, "origin", xyz="0 0 .04", rpy="0 0 0")
        child(child(visual, "geometry"), "box", size=".008 .020 .080")
        # Two separated contact pads provide friction at distinct points along
        # the housing's long axis, rather than a single central contact pair.
        for index, y in enumerate((-0.006, 0.006) if split_contacts else (0,)):
            shape = child(finger, "collision", name=f"contact_pad_{index}")
            child(shape, "origin", xyz=f"0 {y} .04", rpy="0 0 0")
            child(
                child(shape, "geometry"),
                "box",
                size=".008 .008 .080" if split_contacts else ".008 .020 .080",
            )
        if backstop or capture_fences:
            # A back-face fence resists yaw around the jaw closing axis. It
            # retracts with the finger; the open jaw leaves the housing clear.
            # TCP is 80 mm from the root, housing rear face is TCP-Z=-8 mm.
            # Leave 0.3 mm nominal clearance and keep the fence inside +/-13 mm
            # along tool-Y, clear of the nest's housing-end supports and corners.
            fences = (
                (("rear_fence", 0.071), ("front_fence", 0.093))
                if capture_fences
                else (("backstop", 0.0712),)
            )
            for fence_name, z in fences:
                for kind in ("collision", "visual"):
                    fence = child(finger, kind, name=fence_name)
                    child(fence, "origin", xyz=f"{-side * 0.006} 0 {z}", rpy="0 0 0")
                    child(child(fence, "geometry"), "box", size=".012 .026 .001")
        joint = child(robot, "joint", name=prefix + "finger_" + name + "_joint", type="prismatic")
        child(joint, "origin", xyz=f"{side * FINGER_REST_X_M} 0 0", rpy="0 0 0")
        child(joint, "parent", link=prefix + "gripper")
        child(joint, "child", link=prefix + "finger_" + name)
        child(joint, "axis", xyz=f"{side} 0 0")
        child(joint, "limit", lower="0", upper=str(MAX_TRAVEL_M), effort="12", velocity=".03")
        child(joint, "dynamics", damping="2", friction=".1")
