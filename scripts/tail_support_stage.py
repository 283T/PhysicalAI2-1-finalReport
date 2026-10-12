"""Independent vertical cable support fixture; contact only, not a robot arm."""

import math

from build_factory import mass
from build_world import box, child, link, plugin

MODEL = "tail_support_stage"
LINK = "tray"
ORIGIN_Z_M = 0.220
TOP_Z_M = 0.226
TOPIC = "/grip_bench/tail/cmd"


def support_height(lift_command, grip_height, phase):
    """Keep the initial support height during approach, then accompany the lift."""
    if not all(math.isfinite(value) for value in (lift_command, grip_height)):
        raise ValueError("Finite lift and grip heights required")
    if phase not in ("lift", "settle_lift", "hold", "return_to_nest", "settle_return"):
        return 0.0
    return min(0.12, max(0.0, lift_command - grip_height))


def add_tail_stage(world, moving=True):
    model = child(world, "model", name=MODEL)
    if not moving:
        child(model, "static", "true")
    tray = link(model, LINK)
    # The fixed cable cradle begins at X=-70 mm. Keep 5 mm clearance;
    # unlike two overlapping static blocks, a moving tray must not enter it.
    child(tray, "pose", f"-.2375 0 {ORIGIN_Z_M} 0 0 0")
    mass(tray, 0.2)
    box(tray, "tail_support", ".325 .06 .012", "0 0 0 0 0 0", ".3 .45 .5 1")
    sensor = child(tray, "sensor", name="support_contact", type="contact")
    child(sensor, "always_on", "true")
    child(sensor, "topic", "/grip_bench/contacts")
    contact = child(sensor, "contact")
    child(contact, "collision", "tail_support_collision")
    child(contact, "topic", "/grip_bench/contacts")
    if not moving:
        return model
    joint = child(model, "joint", name="support_lift", type="prismatic")
    child(joint, "parent", "world")
    child(joint, "child", LINK)
    axis = child(joint, "axis")
    child(axis, "xyz", "0 0 1")
    limits = child(axis, "limit")
    child(limits, "lower", 0)
    child(limits, "upper", 0.12)
    child(limits, "effort", 100)
    child(limits, "velocity", 0.03)
    plugin(
        model,
        ("joint-position-controller", "JointPositionController"),
        joint_name="support_lift",
        topic=TOPIC,
        p_gain=500,
        d_gain=20,
        i_gain=100,
        i_max=100,
        i_min=-100,
        cmd_max=100,
        cmd_min=-100,
    )
    return model
