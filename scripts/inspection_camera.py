"""Validate ideal camera extrinsics and add an optional diagnostic RGB-D view."""

import copy

import numpy as np
from arm_kinematics import transform


def validate_calibration(calibration):
    position = np.asarray(calibration["position_m"], dtype=float)
    angles = np.asarray(calibration["pose_rpy_rad"], dtype=float)
    optical = np.asarray(calibration["optical_to_world_rotation"], dtype=float)
    if (
        position.shape != (3,)
        or angles.shape != (3,)
        or optical.shape != (3, 3)
        or not all(np.isfinite(value).all() for value in (position, angles, optical))
    ):
        raise ValueError("Finite camera pose and optical rotation required")
    # Gazebo camera axes: forward X, left Y, up Z. Optical: right X, down Y, forward Z.
    native_from_optical = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    expected = transform(rpy=angles)[:3, :3] @ native_from_optical
    if not np.allclose(optical, expected, atol=1e-8):
        raise ValueError("Optical calibration disagrees with the SDF camera pose")
    if calibration.get("frame") != "world":
        raise ValueError("Camera calibration must use the world frame")
    if not 0 < calibration["horizontal_fov_rad"] < np.pi:
        raise ValueError("Invalid camera field of view")
    for name in ("width", "height"):
        if type(calibration[name]) is not int or calibration[name] <= 0:
            raise ValueError("Positive camera resolution required")
    if not isinstance(calibration["topic"], str) or not calibration["topic"].startswith("/"):
        raise ValueError("Absolute camera topic required")
    return calibration


def add_inspection_camera(world, calibration):
    validate_calibration(calibration)
    template = world.find("model[@name='overhead_camera']")
    if template is None or world.find("model[@name='inspection_camera']") is not None:
        raise ValueError("Need one overhead camera and no existing inspection camera")
    camera = copy.deepcopy(template)
    camera.set("name", "inspection_camera")
    for pose in camera.findall("pose"):
        camera.remove(pose)
    camera_link = camera.find("link")
    camera_link.set("name", "inspection_camera_link")
    camera_link.find("pose").text = " ".join(
        map(str, calibration["position_m"] + calibration["pose_rpy_rad"])
    )
    sensor = camera_link.find("sensor")
    for pose in sensor.findall("pose"):
        sensor.remove(pose)
    if sensor.get("type") != "rgbd_camera" or sensor.findtext("topic") == calibration["topic"]:
        raise ValueError("Inspection RGB-D camera needs a distinct topic")
    sensor.set("name", "inspection_rgbd")
    sensor.find("topic").text = calibration["topic"]
    sensor.find("update_rate").text = "1"
    sensor.find("camera/horizontal_fov").text = str(calibration["horizontal_fov_rad"])
    for name in ("width", "height"):
        sensor.find("camera/image/" + name).text = str(calibration[name])
    world.append(camera)
