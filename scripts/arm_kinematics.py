"""URDF-based forward/inverse kinematics, independent of ROS and ground truth."""

import math
import xml.etree.ElementTree as ET

import numpy as np


def quaternion_rotation(q):
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    roll, pitch, yaw = rpy
    matrix = np.eye(4)
    matrix[:3, :3] = (
        axis_rotation([0, 0, 1], yaw)
        @ axis_rotation([0, 1, 0], pitch)
        @ axis_rotation([1, 0, 0], roll)
    )
    matrix[:3, 3] = xyz
    return matrix


def rotation_error(target, current):
    """World-frame SO(3) logarithm, including the 180-degree case."""
    delta = target @ current.T
    angle = math.acos(np.clip((np.trace(delta) - 1) / 2, -1, 1))
    vector = np.array(
        [delta[2, 1] - delta[1, 2], delta[0, 2] - delta[2, 0], delta[1, 0] - delta[0, 1]]
    )
    if angle < 1e-6:
        return vector / 2
    if math.pi - angle < 1e-5:
        values, vectors = np.linalg.eigh((delta + delta.T) / 2)
        axis = vectors[:, np.argmax(values)]
        if np.dot(axis, vector) < 0:
            axis = -axis
        return axis * angle
    return vector * angle / (2 * math.sin(angle))


class ArmKinematics:
    def __init__(self, urdf_path, tip_link, base_pose=(0, 0, 0, 0, 0, 0), tip_offset=(0, 0, 0.08)):
        robot = ET.parse(urdf_path).getroot()
        by_child = {joint.find("child").get("link"): joint for joint in robot.findall("joint")}
        chain = []
        child = tip_link
        while child in by_child:
            joint = by_child[child]
            chain.append(joint)
            child = joint.find("parent").get("link")
        if not chain or child != "world":
            raise ValueError("Tool chain must reach the URDF world link")
        self.chain = []
        self.names, limits = [], []
        self._joint_rotations = []
        self._identity_motion = np.eye(4)
        for joint in reversed(chain):
            origin = joint.find("origin")
            xyz = (
                [float(v) for v in origin.get("xyz", "0 0 0").split()]
                if origin is not None
                else [0] * 3
            )
            rpy = (
                [float(v) for v in origin.get("rpy", "0 0 0").split()]
                if origin is not None
                else [0] * 3
            )
            kind = joint.get("type")
            if kind not in ("fixed", "revolute"):
                raise ValueError("Unsupported joint type: " + str(kind))
            axis = None
            if kind == "revolute":
                axis = np.array([float(v) for v in joint.find("axis").get("xyz").split()])
                axis /= np.linalg.norm(axis)
                # URDF axes are constant. Prepare Rodrigues coefficients once
                # instead of normalizing axes and allocating identities per FK.
                x, y, z = axis / np.linalg.norm(axis)
                skew = np.zeros((4, 4))
                skew[:3, :3] = [[0, -z, y], [z, 0, -x], [-y, x, 0]]
                self._joint_rotations.append((skew, skew @ skew))
                self.names.append(joint.get("name"))
                limit = joint.find("limit")
                limits.append([float(limit.get("lower")), float(limit.get("upper"))])
            self.chain.append((transform(xyz, rpy), axis))
        self.limits = np.asarray(limits)
        self.base = transform(base_pose[:3], base_pose[3:])
        self.tool = transform(tip_offset)

    def forward(self, q, *, jacobian=False):
        q = np.asarray(q, dtype=float)
        if q.shape != (len(self.names),) or not np.isfinite(q).all():
            raise ValueError("Joint vector size or values invalid")
        matrix = self.base.copy()
        origins, axes = [], []
        index = 0
        for origin, axis in self.chain:
            matrix = matrix @ origin
            if axis is not None:
                if jacobian:
                    origins.append(matrix[:3, 3].copy())
                    axes.append(matrix[:3, :3] @ axis)
                skew, square = self._joint_rotations[index]
                motion = (
                    self._identity_motion
                    + math.sin(q[index]) * skew
                    + (1 - math.cos(q[index])) * square
                )
                matrix = matrix @ motion
                index += 1
        matrix = matrix @ self.tool
        if not jacobian:
            return matrix
        derivative = np.array(
            [
                np.r_[np.cross(axis, matrix[:3, 3] - origin), axis]
                for origin, axis in zip(origins, axes)
            ]
        ).T
        return matrix, derivative

    def inverse(
        self, target, seed=None, attempts=12, *, position_tolerance=0.001, angle_tolerance=0.015
    ):
        target = np.asarray(target, dtype=float)
        if target.shape != (4, 4) or not np.isfinite(target).all():
            raise ValueError("Target must be a finite 4x4 pose")
        if (
            not np.allclose(target[3], [0, 0, 0, 1])
            or not np.allclose(target[:3, :3].T @ target[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(target[:3, :3]), 1)
        ):
            raise ValueError("Target must be a rigid transform")
        if (
            not np.isfinite([position_tolerance, angle_tolerance]).all()
            or min(position_tolerance, angle_tolerance) <= 0
        ):
            raise ValueError("Positive finite IK tolerances required")
        if attempts < 1:
            raise ValueError("At least one IK attempt is required")
        rng = np.random.default_rng(0)
        seeds = [np.zeros(len(self.names)) if seed is None else np.asarray(seed, dtype=float)]
        seeds += [
            rng.uniform(self.limits[:, 0] * 0.7, self.limits[:, 1] * 0.7)
            for _ in range(attempts - 1)
        ]
        best = None
        weight = np.array([1, 1, 1, 0.2, 0.2, 0.2])
        for initial in seeds:
            q = np.clip(initial, self.limits[:, 0] + 1e-8, self.limits[:, 1] - 1e-8)
            for _ in range(220):
                pose, jacobian = self.forward(q, jacobian=True)
                error = np.r_[
                    target[:3, 3] - pose[:3, 3], rotation_error(target[:3, :3], pose[:3, :3])
                ]
                position_error, angle_error = np.linalg.norm(error[:3]), np.linalg.norm(error[3:])
                score = np.linalg.norm(error * weight)
                if best is None or score < best["score"]:
                    best = {
                        "q_rad": q.tolist(),
                        "position_error_m": float(position_error),
                        "angle_error_rad": float(angle_error),
                        "score": float(score),
                        "success": bool(
                            position_error < position_tolerance and angle_error < angle_tolerance
                        ),
                    }
                if best["success"]:
                    # Equivalent revolute angles avoid unnecessary whole turns within URDF limits.
                    reference = seeds[0]
                    for index, angle in enumerate(best["q_rad"]):
                        equivalents = [
                            angle + 2 * math.pi * turn
                            for turn in range(-2, 3)
                            if self.limits[index, 0]
                            <= angle + 2 * math.pi * turn
                            <= self.limits[index, 1]
                        ]
                        best["q_rad"][index] = min(
                            equivalents, key=lambda value: abs(value - reference[index])
                        )
                    return best
                j = jacobian * weight[:, None]
                step = j.T @ np.linalg.solve(j @ j.T + 0.003**2 * np.eye(6), error * weight)
                step *= min(1.0, 0.25 / max(np.max(np.abs(step)), 1e-12))
                improved = False
                for fraction in (1.0, 0.5, 0.25, 0.1):
                    candidate = np.clip(
                        q + step * fraction, self.limits[:, 0] + 1e-8, self.limits[:, 1] - 1e-8
                    )
                    pose = self.forward(candidate)
                    residual = np.r_[
                        target[:3, 3] - pose[:3, 3], rotation_error(target[:3, :3], pose[:3, :3])
                    ]
                    if np.linalg.norm(residual * weight) < score:
                        q, improved = candidate, True
                        break
                if not improved:
                    break
        return best
