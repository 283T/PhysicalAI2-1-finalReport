"""Offline permanent finger candidate: broad wire pads above a narrow connector tip."""

import copy
import math
import xml.etree.ElementTree as ET


def stepped_profile(robot, prefix, shoulder_z=0.067, tip_span=0.020):
    """Return a new URDF tree, never change a running model or the source tree."""
    if not (math.isfinite(shoulder_z) and 0.050 <= shoulder_z <= 0.070):
        raise ValueError("Shoulder must leave at least 10 mm of tip")
    if not (math.isfinite(tip_span) and 0.016 <= tip_span <= 0.024):
        raise ValueError("Connector tip span must be between 16 and 24 mm")
    robot = copy.deepcopy(robot)
    for side in ("left", "right"):
        finger = robot.find(f"link[@name='{prefix}finger_{side}']")
        if finger is None:
            raise ValueError("Known pair of bare wire fingers required")
        pads = finger.findall("collision")
        if len(pads) != 2 or any(not p.get("name", "").startswith("contact_pad") for p in pads):
            raise ValueError("Exactly two bare split wire pads required")
        original = []
        for pad in pads:
            size = [float(v) for v in pad.find("geometry/box").get("size").split()]
            origin = pad.find("origin")
            xyz = [float(v) for v in origin.get("xyz").split()]
            if (
                origin.get("rpy", "0 0 0") != "0 0 0"
                or len(size) != 3
                or len(xyz) != 3
                or not all(math.isfinite(v) for v in [*size, *xyz])
                or max(abs(a - b) for a, b in zip(size, (0.013, 0.020, 0.080))) > 1e-9
                or abs(xyz[2] - 0.040) > 1e-9
                or abs(abs(xyz[1]) - 0.015) > 1e-9
            ):
                raise ValueError("Expected unrotated 50 mm extended wire pad profile")
            original.append((size, xyz))
        old_mass = float(finger.find("inertial/mass").get("value"))
        if not math.isfinite(old_mass) or old_mass <= 0:
            raise ValueError("Finite positive source finger mass required")
        if abs(sum(xyz[1] for _, xyz in original)) > 1e-9:
            raise ValueError("Symmetric broad contact pads required")
        density = old_mass / sum(math.prod(size) for size, _ in original)
        pieces = []
        for size, xyz in original:
            sign = 1 if xyz[1] > 0 else -1
            pieces.extend(
                [
                    ([size[0], size[1], shoulder_z], [xyz[0], xyz[1], shoulder_z / 2]),
                    (
                        [size[0], tip_span * 0.4, 0.080 - shoulder_z],
                        [xyz[0], sign * tip_span * 0.3, (0.080 + shoulder_z) / 2],
                    ),
                ]
            )
        for shape in [*finger.findall("visual"), *pads]:
            finger.remove(shape)
        masses = [density * math.prod(size) for size, _ in pieces]
        mass = sum(masses)
        center = [sum(m * xyz[k] for m, (_, xyz) in zip(masses, pieces)) / mass for k in range(3)]
        tensor = [[0.0] * 3 for _ in range(3)]
        for index, (m, (size, xyz)) in enumerate(zip(masses, pieces)):
            delta = [a - b for a, b in zip(xyz, center)]
            for i in range(3):
                tensor[i][i] += m * sum(
                    size[k] ** 2 / 12 + delta[k] ** 2 for k in range(3) if k != i
                )
                for j in range(i + 1, 3):
                    tensor[i][j] -= m * delta[i] * delta[j]
            for kind in ("collision", "visual"):
                shape = ET.SubElement(finger, kind, name=f"contact_pad_step_{index}")
                ET.SubElement(shape, "origin", xyz=" ".join(map(str, xyz)), rpy="0 0 0")
                ET.SubElement(
                    ET.SubElement(shape, "geometry"), "box", size=" ".join(map(str, size))
                )
        inertial = finger.find("inertial")
        inertial.find("origin").set("xyz", " ".join(map(str, center)))
        inertial.find("mass").set("value", str(mass))
        inertia = inertial.find("inertia")
        for name, i, j in (
            ("ixx", 0, 0),
            ("iyy", 1, 1),
            ("izz", 2, 2),
            ("ixy", 0, 1),
            ("ixz", 0, 2),
            ("iyz", 1, 2),
        ):
            inertia.set(name, str(tensor[i][j]))
    return robot
