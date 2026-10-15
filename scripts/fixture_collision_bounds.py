"""Box collision screening against fixed workcell fixtures in the generated SDF."""

import xml.etree.ElementTree as ET

import numpy as np
from arm_collision_bounds import intersects
from arm_kinematics import transform
from cartesian_path import interpolate_joints


def sdf_pose(element):
    pose = element.find("pose")
    if pose is not None and (pose.get("relative_to") or pose.get("rotation_format")):
        raise ValueError("Fixture poses must use parent-relative Euler coordinates")
    values = [float(v) for v in (pose.text if pose is not None else "0 0 0 0 0 0").split()]
    if len(values) != 6 or not np.isfinite(values).all():
        raise ValueError("Invalid fixture pose")
    return transform(values[:3], values[3:])


class FixtureBounds:
    def __init__(self, world, *, include_carrier=False):
        root = ET.parse(world).getroot().find("world")
        model = root.find("model[@name='fixtures']")
        if model is None or model.findtext("static") != "true":
            raise ValueError("A static fixtures model is required")
        self.shapes = []
        models = [model]
        if include_carrier:
            carrier = root.find("model[@name='conveyor_carrier']")
            if carrier is None:
                raise ValueError("Stopped conveyor carrier required")
            models.append(carrier)
        for model in models:
            for link in model.findall("link"):
                parent = sdf_pose(model) @ sdf_pose(link)
                for collision in link.findall("collision"):
                    box = collision.find("geometry/box/size")
                    if box is None:
                        raise ValueError("Unsupported fixture collision shape")
                    half = np.asarray([float(v) for v in box.text.split()]) / 2
                    if half.shape != (3,) or not np.isfinite(half).all() or min(half) <= 0:
                        raise ValueError("Invalid fixture box")
                    pose = parent @ sdf_pose(collision)
                    self.shapes.append(
                        (
                            ("carrier/" if model.get("name") == "conveyor_carrier" else "")
                            + collision.get("name"),
                            (pose[:3, 3], pose[:3, :3], half),
                        )
                    )
        if not self.shapes:
            raise ValueError("No fixture collision boxes")
        self.centers = np.array([box[0] for _, box in self.shapes])
        self.radii = np.array([np.linalg.norm(box[2] + 0.005) for _, box in self.shapes])

    def first_overlap(self, bounds, joints, margin=0.005):
        for name, arm_bounds in bounds.items():
            for link, box in arm_bounds.boxes(joints[name]):
                nearby = np.linalg.norm(
                    self.centers - box[0], axis=1
                ) <= self.radii + np.linalg.norm(box[2] + 0.005)
                for index in np.flatnonzero(nearby):
                    fixture, shape = self.shapes[index]
                    if intersects(box, shape, margin=margin):
                        return [name + "/" + link, fixture]
        return None

    def screen(self, bounds, paths, margin=0.005):
        if not np.isfinite(margin) or margin < 0 or margin > 0.005:
            raise ValueError("Fixture margin must be finite and between 0 and 5 mm")
        samples = max(
            81,
            1
            + max(
                int(np.ceil((len(path) - 1) * np.max(np.abs(np.diff(path, axis=0))) / 0.02))
                for path in paths.values()
            ),
        )
        for fraction in np.linspace(0, 1, samples):
            joints = {name: interpolate_joints(path, fraction) for name, path in paths.items()}
            overlap = self.first_overlap(bounds, joints, margin)
            if overlap:
                return {"clear": False, "fraction": float(fraction), "shapes": overlap}
        return {"clear": True, "samples": samples, "margin_per_box_m": margin}
