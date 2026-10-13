"""ROS-independent RGB-D decoding, timestamp pairing and stability checks."""

import cv2
import numpy as np
from arm_kinematics import rotation_error

PLUG_IDS = frozenset(("P1", "P2", "P3"))


def stamp_ns(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


def image_array(message):
    """Decode ROS-compatible Image fields, including padded rows and endianness."""
    endian = ">" if message.is_bigendian else "<"
    if message.encoding in ("rgb8", "bgr8"):
        data = np.ndarray(
            (message.height, message.width, 3),
            dtype="u1",
            buffer=bytes(message.data),
            strides=(message.step, 3, 1),
        ).copy()
        return cv2.cvtColor(data, cv2.COLOR_RGB2BGR) if message.encoding == "rgb8" else data
    if message.encoding == "32FC1":
        return np.ndarray(
            (message.height, message.width),
            dtype=endian + "f4",
            buffer=bytes(message.data),
            strides=(message.step, 4),
        ).astype(np.float32)
    raise ValueError("Unsupported image encoding: " + message.encoding)


class SyncedFrames:
    """Bounded buffer yielding the newest exact RGB/depth pair once per timestamp."""

    def __init__(self, capacity=10):
        self.capacity = capacity
        self.images = {"rgb": {}, "depth": {}}
        self.last_stamp = -1

    def add(self, message, kind):
        images = self.images[kind]
        images[stamp_ns(message)] = message
        while len(images) > self.capacity:
            del images[min(images)]

    def take_latest(self):
        shared = set(self.images["rgb"]) & set(self.images["depth"])
        if not shared or max(shared) <= self.last_stamp:
            return None
        current = max(shared)
        rgb, depth = self.images["rgb"][current], self.images["depth"][current]
        self.last_stamp = current
        for images in self.images.values():
            for key in list(images):
                if key <= current:
                    del images[key]
        return current, rgb, depth


class StableConnectors:
    """Require all unique terminals and consecutive small position/directed-yaw changes."""

    def __init__(
        self,
        required_frames=3,
        position_limit=0.003,
        yaw_limit=0.08,
        require_6d=False,
        required_ids=PLUG_IDS,
    ):
        self.required_ids = frozenset(required_ids)
        if not self.required_ids or not self.required_ids <= PLUG_IDS:
            raise ValueError("Known nonempty required terminal IDs needed")
        self.require_6d = require_6d
        self.previous_rotations = None
        self.required_frames = required_frames
        self.position_limit = position_limit
        self.yaw_limit = yaw_limit
        self.count = 0
        self.previous = None
        self.previous_yaws = None

    def update(self, detections):
        if (
            len(detections) != len(self.required_ids)
            or {row["plug_id"] for row in detections} != self.required_ids
        ):
            self.count, self.previous, self.previous_yaws = 0, None, None
            return False
        if self.require_6d and not all("pose_6d" in row for row in detections):
            self.count, self.previous, self.previous_rotations = 0, None, None
            return False
        ordered = sorted(detections, key=lambda row: row["plug_id"])
        positions = np.array(
            [
                row["pose_6d"]["center_xyz_m"] if self.require_6d else row["surface_xyz_m"]
                for row in ordered
            ]
        )
        rotations = (
            [np.asarray(row["pose_6d"]["rotation_world_from_connector"]) for row in ordered]
            if self.require_6d
            else None
        )
        yaws = np.array([row["yaw_rad"] for row in ordered])
        consistent = False
        if self.previous is not None:
            yaw_delta = yaws - self.previous_yaws
            yaw_change = (
                max(
                    np.linalg.norm(rotation_error(current, previous))
                    for current, previous in zip(rotations, self.previous_rotations)
                )
                if self.require_6d
                else np.max(np.abs(np.arctan2(np.sin(yaw_delta), np.cos(yaw_delta))))
            )
            consistent = (
                np.max(np.linalg.norm(positions - self.previous, axis=1)) < self.position_limit
                and yaw_change < self.yaw_limit
            )
        self.count = self.count + 1 if consistent else 1
        self.previous, self.previous_yaws = positions, yaws
        self.previous_rotations = rotations
        return self.count >= self.required_frames
