"""Passive regrasp nest: housing-end supports and low, open pin guides."""

import math

import numpy as np
from build_world import box


def add_regrasp_nest(
    fixtures_link, center=(0, 0), rim=0.22, yaw=0.0, channel_width=0.008, pin_relief=False
):
    """Support housing ends outside the central 20 mm finger contact band."""
    if len(center) != 2 or not np.isfinite([*center, rim, yaw]).all() or rim <= 0.10:
        raise ValueError("Finite nest position/yaw and rim above 100 mm required")
    if not math.isfinite(channel_width) or not 0.004 < channel_width <= 0.008:
        raise ValueError("Pin channel must be wider than 4 mm and at most 8 mm")

    # Supports end on the housing underside. Guides only engage the pin tips,
    # leaving the lower side-grip finger's opening path above them.
    def shape(name, size, xyz):
        x, y, z = xyz
        cx = center[0] + x * math.cos(yaw) - y * math.sin(yaw)
        cy = center[1] + x * math.sin(yaw) + y * math.cos(yaw)
        box(
            fixtures_link,
            "regrasp_" + name,
            " ".join(map(str, size)),
            f"{cx} {cy} {z} 0 0 {yaw}",
            ".45 .45 .5 1",
        )

    base_top = rim - 0.045
    shape("base", [0.06, 0.04, base_top - 0.04], [0, 0, (base_top + 0.04) / 2])
    # Support the proximal cable so its weight does not tilt the captured pins.
    # The open upper surface allows the cable to leave with a vertical lift.
    cradle_top = rim + 0.006
    shape("cable_cradle", [0.04, 0.016, cradle_top - 0.04], [-0.05, 0, (cradle_top + 0.04) / 2])
    for sign in (-1, 1):
        if pin_relief:
            # Keep the housing supported at both corners; let a tilted pin pass
            # through the central 10 mm opening rather than hit a crossbar.
            for side in (-1, 1):
                shape(
                    f"end_support_{sign}_{side}",
                    [0.002, 0.007, rim - base_top],
                    [sign * 0.019, side * 0.0085, (rim + base_top) / 2],
                )
        else:
            shape(
                f"end_support_{sign}",
                [0.002, 0.024, rim - base_top],
                [sign * 0.019, 0, (rim + base_top) / 2],
            )
        # Corner stops restrain housing tilt from cable torque without occupying
        # the central +/-10 mm contact band used by either gripper orientation.
        stop_bottom, stop_top = rim - 0.002, rim + 0.014
        for side in (-1, 1):
            # Leave a 10 mm central outlet for the 8 mm cable at connector -X.
            shape(
                f"end_stop_{sign}_{side}",
                [0.002, 0.005, stop_top - stop_bottom],
                [sign * 0.022, side * 0.0075, (stop_top + stop_bottom) / 2],
            )
            shape(
                f"corner_{sign}_{side}",
                [0.004, 0.002, stop_top - stop_bottom],
                [sign * 0.018, side * 0.012, (stop_top + stop_bottom) / 2],
            )
        # Low guides leave room for the side finger's opening stroke.
        top = rim - 0.035
        wall_center = channel_width / 2 + 0.001
        for side in (-1, 1):
            shape(
                f"pin_{sign}_x_{side}",
                [0.002, channel_width + 0.004, top - base_top],
                [sign * 0.01 + side * wall_center, 0, (top + base_top) / 2],
            )
            shape(
                f"pin_{sign}_y_{side}",
                [channel_width, 0.002, top - base_top],
                [sign * 0.01, side * wall_center, (top + base_top) / 2],
            )
    return {
        "housing_center_m": [*center, rim + 0.01],
        "yaw_rad": yaw,
        "pin_channel_width_m": channel_width,
        "guide_top_m": rim - 0.035,
        "housing_stop_clearance_m": 0.001,
        "cable_outlet_width_m": 0.010,
        "cable_cradle_top_m": cradle_top,
        "pin_relief": pin_relief,
        "scope": "Passive prototype nest; no latch, clamp or grasp transition",
    }


def add_nest_to_world(world, center=(0.08, -0.04), rim=0.22, yaw=0.0):
    fixtures = world.find("model[@name='fixtures']/link")
    if fixtures is None:
        raise ValueError("Missing factory fixtures link")
    if any(c.get("name", "").startswith("regrasp_") for c in fixtures):
        raise ValueError("Regrasp nest already present")
    return add_regrasp_nest(fixtures, center, rim, yaw)
