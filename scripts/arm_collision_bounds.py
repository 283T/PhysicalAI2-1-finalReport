"""Conservative oriented boxes enclosing URDF collision shapes, for inter-arm screening."""

import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from arm_kinematics import axis_rotation, transform
from cartesian_path import interpolate_joints
from gripper_model import MAX_TRAVEL_M
from project_paths import ROOT


def intersects(first, second, margin=0.005):
    """Separating-axis test for two oriented boxes, inflated by margin per box."""
    ca, axes_a, ha = first
    cb, axes_b, hb = second
    a, b = ha + margin, hb + margin
    rotation = axes_a.T @ axes_b
    absolute = np.abs(rotation) + 1e-10
    delta = axes_a.T @ (cb - ca)
    if np.any(np.abs(delta) > a + absolute @ b):
        return False
    if np.any(np.abs(delta @ rotation) > b + a @ absolute):
        return False
    for i in range(3):
        for j in range(3):
            i1, i2, j1, j2 = (i + 1) % 3, (i + 2) % 3, (j + 1) % 3, (j + 2) % 3
            distance = abs(delta[i2] * rotation[i1, j] - delta[i1] * rotation[i2, j])
            radius = a[i1] * absolute[i2, j] + a[i2] * absolute[i1, j]
            radius += b[j1] * absolute[i, j2] + b[j2] * absolute[i, j1]
            if distance > radius:
                return False
    return True


def origin(element):
    if element is None:
        return np.eye(4)
    return transform(
        [float(v) for v in element.get("xyz", "0 0 0").split()],
        [float(v) for v in element.get("rpy", "0 0 0").split()],
    )


def mesh_bounds(filename, scale):
    path = Path(filename.removeprefix("file://"))
    if not path.exists() and str(path).startswith("/workspace/"):
        path = ROOT / path.relative_to("/workspace")
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise ValueError("Collision screening currently requires binary STL: " + str(path))
    records = np.ndarray(
        (count,),
        dtype=np.dtype(
            [("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]
        ),
        buffer=data,
        offset=84,
    )
    vertices = records["vertices"].reshape(-1, 3) * scale
    return vertices.min(axis=0), vertices.max(axis=0)


class ArmCollisionBounds:
    def __init__(self, urdf, arm):
        self.arm = arm
        robot = ET.parse(urdf).getroot()
        self.joints = robot.findall("joint")
        self.prismatic_links = {
            j.find("child").get("link") for j in self.joints if j.get("type") == "prismatic"
        }
        children = {joint.find("child").get("link") for joint in self.joints}
        self.roots = [
            link.get("name") for link in robot.findall("link") if link.get("name") not in children
        ]
        self.shapes = []
        for link in robot.findall("link"):
            for collision in link.findall("collision"):
                geometry = collision.find("geometry")
                mesh = geometry.find("mesh")
                box = geometry.find("box")
                if mesh is not None:
                    low, high = mesh_bounds(
                        mesh.get("filename"),
                        np.array([float(v) for v in mesh.get("scale", "1 1 1").split()]),
                    )
                elif box is not None:
                    high = np.array([float(v) for v in box.get("size").split()]) / 2
                    low = -high
                else:
                    raise ValueError("Unsupported arm collision shape")
                local = origin(collision.find("origin"))
                center = local.copy()
                center[:3, 3] += local[:3, :3] @ ((high + low) / 2)
                self.shapes.append((link.get("name"), center, (high - low) / 2))

    def boxes(self, q, finger_travel=MAX_TRAVEL_M / 2, *, finger_intervals=None):
        intervals = {} if finger_intervals is None else finger_intervals
        finger_expansion = {}
        coordinates = dict(zip(self.arm.names, q))
        frames = {name: self.arm.base for name in self.roots}
        pending = list(self.joints)
        while pending:
            progress = False
            for joint in list(pending):
                parent = joint.find("parent").get("link")
                if parent not in frames:
                    continue
                pose = frames[parent] @ origin(joint.find("origin"))
                kind = joint.get("type")
                if kind != "fixed":
                    axis = np.array([float(v) for v in joint.find("axis").get("xyz").split()])
                    axis /= np.linalg.norm(axis)
                    motion = np.eye(4)
                    if kind == "revolute":
                        motion[:3, :3] = axis_rotation(axis, coordinates[joint.get("name")])
                    elif kind == "prismatic":
                        interval = np.asarray(
                            intervals.get(joint.get("name"), [0, MAX_TRAVEL_M]), dtype=float
                        )
                        if (
                            interval.shape != (2,)
                            or not np.isfinite(interval).all()
                            or interval[0] > interval[1]
                            or interval[0] < -0.0005
                            or interval[1] > MAX_TRAVEL_M + 0.0005
                        ):
                            raise ValueError("Bounded measured finger interval required")
                        midpoint = (
                            float(interval.mean())
                            if joint.get("name") in intervals
                            else finger_travel
                        )
                        motion[:3, 3] = axis * midpoint
                        finger_expansion[joint.find("child").get("link")] = (
                            axis,
                            (interval[1] - interval[0]) / 2,
                        )
                    else:
                        raise ValueError("Unsupported arm joint type")
                    pose = pose @ motion
                frames[joint.find("child").get("link")] = pose
                pending.remove(joint)
                progress = True
            if not progress:
                raise ValueError("Broken URDF collision tree")
        result = []
        for name, local, half in self.shapes:
            pose = frames[name] @ local
            if name in self.prismatic_links:
                # The midpoint box encloses the whole 0..20 mm finger stroke.
                axis, expansion = finger_expansion[name]
                half = half + np.abs(local[:3, :3].T @ axis) * expansion
            result.append((name, (pose[:3, 3], pose[:3, :3], half)))
        return result


class FingerStateBounds:
    """A screened interval; execution must stop if a finger leaves it."""

    def __init__(self, bounds, intervals):
        self.bounds, self.intervals = bounds, intervals

    def boxes(self, q):
        return self.bounds.boxes(q, finger_intervals=self.intervals)


def finger_collision_intervals(fingers, *, full_stroke=False):
    values = np.asarray(fingers, dtype=float)
    if (
        values.shape != (2,)
        or not np.isfinite(values).all()
        or np.any(values < -0.0005)
        or np.any(values > 0.0205)
    ):
        raise ValueError("Two finite finger observations within mechanical tolerances required")
    if full_stroke:
        return np.tile([-0.0005, 0.0205], (2, 1))
    return np.column_stack(
        (np.maximum(values - 0.0005, -0.0005), np.minimum(values + 0.0005, 0.0205))
    )


def first_overlap(bounds, joints):
    names = list(joints)
    a, b = names
    left, right = bounds[a].boxes(joints[a]), bounds[b].boxes(joints[b])
    for na, first in left:
        for nb, second in right:
            # Cheap sphere rejection before the oriented-box test.
            if np.linalg.norm(first[0] - second[0]) > np.linalg.norm(
                first[2] + 0.005
            ) + np.linalg.norm(second[2] + 0.005):
                continue
            if intersects(first, second):
                return [na, nb]
    return None


def with_parked_arms(paths, parked):
    """Include measured idle joints as constant paths during collision screening."""
    if set(paths) & set(parked):
        raise ValueError("An arm cannot be moving and parked")
    result = dict(paths)
    for name, values in parked.items():
        q = np.asarray(values, dtype=float)
        if q.ndim != 1 or not len(q) or not np.isfinite(q).all():
            raise ValueError("Finite parked joint observations required")
        result[name] = np.repeat(q[None, :], 2, axis=0)
    return result


def screen_paths(bounds, paths, samples=81):
    # Bound the commanded joint change between samples to 0.02 rad.
    samples = max(
        samples,
        1
        + max(
            int(np.ceil((len(path) - 1) * np.max(np.abs(np.diff(path, axis=0))) / 0.02))
            for path in paths.values()
        ),
    )
    for fraction in np.linspace(0, 1, samples):
        joints = {name: interpolate_joints(path, fraction) for name, path in paths.items()}
        overlap = first_overlap(bounds, joints)
        if overlap:
            return {"clear": False, "fraction": float(fraction), "links": overlap}
    return {"clear": True, "samples": samples, "margin_per_box_m": 0.005}
