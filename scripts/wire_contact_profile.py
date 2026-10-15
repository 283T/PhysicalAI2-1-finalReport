"""Check a deeper support grasp against the actual, permanent finger geometry."""

import hashlib
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from gripper_model import measured_rest_gap


def validate_deep_wire_profile(urdf, prefix, depth_m):
    """Geometry permission only; continuous paths, closure and grip remain unproven."""
    if not math.isfinite(depth_m) or not 0.012 < depth_m <= 0.024:
        raise ValueError("Model-checked deep grasp must be within (12, 24] mm")
    urdf = Path(urdf)
    robot = ET.parse(urdf).getroot()
    rest_gap = measured_rest_gap(urdf, prefix)
    if not 0 < rest_gap < 0.008:
        raise ValueError("Fingers must be able to close around the 8 mm wire")
    # Wire cross section is 8 mm. Keep 1 mm on each side of its normal extent;
    # square face alignment is still required by the image-derived wire frame.
    center_z = 0.080 - depth_m
    required_z = [center_z - 0.005, center_z + 0.005]
    report = {
        "scope": "Static bare finger contact geometry only; no wire identity, path, closure, force or holding success",
        "urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
        "tip_depth_m": depth_m,
        "wire_cross_section_m": 0.008,
        "normal_margin_m": 0.001,
        "required_local_z_interval_m": required_z,
        "rest_gap_m": rest_gap,
        "fingers": {},
    }
    for side in ("left", "right"):
        finger = robot.find(f"link[@name='{prefix}finger_{side}']")
        if finger is None:
            raise ValueError("Known two-finger profile required")
        active = []
        for shape in finger.findall("collision"):
            if not shape.get("name", "").startswith("contact_pad"):
                raise ValueError("Deeper wire support requires a bare contact-pad profile")
            pose = shape.find("origin")
            box = shape.find("geometry/box")
            if pose is None or box is None or pose.get("rpy", "0 0 0") != "0 0 0":
                raise ValueError("Unrotated box contact geometry required")
            xyz = list(map(float, pose.get("xyz").split()))
            size = list(map(float, box.get("size").split()))
            if (
                len(xyz) != 3
                or len(size) != 3
                or not all(map(math.isfinite, xyz + size))
                or min(size) <= 0
            ):
                raise ValueError("Finite positive contact geometry required")
            z_range = [xyz[2] - size[2] / 2, xyz[2] + size[2] / 2]
            if z_range[0] <= required_z[0] and z_range[1] >= required_z[1]:
                active.append([xyz[1] - size[1] / 2, xyz[1] + size[1] / 2])
        active.sort()
        if (
            len(active) != 2
            or min(b - a for a, b in active) < 0.018
            or active[0][0] > -0.025 + 1e-9
            or active[0][1] >= 0
            or active[1][0] <= 0
            or active[1][1] < 0.025 - 1e-9
        ):
            raise ValueError("Full wire cross section must be supported by both broad split pads")
        report["fingers"][side] = {"active_local_y_intervals_m": active}
    return report
