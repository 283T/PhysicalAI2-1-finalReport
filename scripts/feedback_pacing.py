"""Advance a screened joint path only while robot tool feedback tracks commands."""

import numpy as np
from arm_kinematics import rotation_error
from cartesian_path import interpolate_joints


def pose_from_feedback(state):
    pose = np.eye(4)
    pose[:3, :3], pose[:3, 3] = state["tcp_rotation_world"], state["tcp"]
    if not np.isfinite(pose).all() or not np.allclose(
        pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-6
    ):
        raise RuntimeError("Invalid robot tool feedback for trajectory pacing")
    return pose


class FeedbackPacer:
    def __init__(self, arms, paths, duration, stamp):
        if not np.isfinite([duration, stamp]).all() or duration <= 0 or not paths:
            raise ValueError("Finite positive duration, timestamp and nonempty paths required")
        if any(
            np.asarray(p).ndim != 2 or len(p) < 2 or not np.isfinite(p).all()
            for p in paths.values()
        ):
            raise ValueError("Finite joint paths with at least two waypoints required")
        self.arms = {name: arms[name] for name in paths}
        self.paths, self.duration, self.last_stamp = paths, duration, stamp
        self.fraction = 0.0
        self.summary = {
            "policy": "Bounded path fraction; wait for both tools and their relative frame",
            "position_gate_m": 0.001,
            "angle_gate_rad": 0.01,
            "joint_gate_rad": 0.01,
            "relative_position_gate_m": 0.0015,
            "relative_angle_gate_rad": 0.015,
            "maximum_command_translation_step_m": 0.0005,
            "maximum_command_rotation_step_rad": 0.003,
            "maximum_command_joint_step_rad": 0.01,
            "advanced_samples": 0,
            "paused_samples": 0,
            "maximum_tracking_position_m": 0.0,
            "maximum_tracking_angle_rad": 0.0,
            "maximum_tracking_joint_error_rad": 0.0,
            "maximum_relative_tracking_position_m": 0.0,
            "maximum_relative_tracking_angle_rad": 0.0,
            "final_fraction": 0.0,
        }

    def commands(self, fraction):
        return {name: interpolate_joints(path, fraction) for name, path in self.paths.items()}

    def update(self, row):
        elapsed = row["sim_time_s"] - self.last_stamp
        current = self.commands(self.fraction)
        if elapsed <= 0:
            return current
        self.last_stamp = row["sim_time_s"]
        commanded = {name: self.arms[name].forward(q) for name, q in current.items()}
        observed = {name: pose_from_feedback(row["arms"][name]) for name in current}
        errors = [
            (
                np.linalg.norm(observed[n][:3, 3] - commanded[n][:3, 3]),
                np.linalg.norm(rotation_error(observed[n][:3, :3], commanded[n][:3, :3])),
            )
            for n in current
        ]
        position, angle = (float(max(e[i] for e in errors)) for i in (0, 1))
        joint_error = float(
            max(np.max(np.abs(np.array(row["arms"][n]["q_rad"]) - current[n])) for n in current)
        )
        if not np.isfinite(joint_error):
            raise RuntimeError("Invalid robot joint feedback for trajectory pacing")
        relative_position, relative_angle = 0.0, 0.0
        if len(current) == 2:
            a, b = current
            measured = np.linalg.inv(observed[a]) @ observed[b]
            target = np.linalg.inv(commanded[a]) @ commanded[b]
            relative_position = float(np.linalg.norm(measured[:3, 3] - target[:3, 3]))
            relative_angle = float(np.linalg.norm(rotation_error(measured[:3, :3], target[:3, :3])))
        summary = self.summary
        for name, value in (
            ("tracking_position_m", position),
            ("tracking_angle_rad", angle),
            ("tracking_joint_error_rad", joint_error),
            ("relative_tracking_position_m", relative_position),
            ("relative_tracking_angle_rad", relative_angle),
        ):
            summary["maximum_" + name] = max(summary["maximum_" + name], value)
        if (
            position > summary["position_gate_m"]
            or angle > summary["angle_gate_rad"]
            or joint_error > summary["joint_gate_rad"]
            or relative_position > summary["relative_position_gate_m"]
            or relative_angle > summary["relative_angle_gate_rad"]
        ):
            summary["paused_samples"] += 1
            return current
        candidate = min(1.0, self.fraction + elapsed / self.duration)
        for _ in range(16):
            next_commands = self.commands(candidate)
            valid = True
            for name, q in next_commands.items():
                pose = self.arms[name].forward(q)
                old = commanded[name]
                valid &= (
                    np.linalg.norm(pose[:3, 3] - old[:3, 3]) <= 0.0005
                    and np.linalg.norm(rotation_error(pose[:3, :3], old[:3, :3])) <= 0.003
                    and np.max(np.abs(q - current[name])) <= 0.01
                )
            if valid:
                self.fraction = candidate
                summary["advanced_samples"] += 1
                summary["final_fraction"] = self.fraction
                return next_commands
            candidate = (candidate + self.fraction) / 2
        raise RuntimeError("No bounded command step in feedback-paced trajectory")
