"""Discrete carried-part fixture screening from intrinsic collision geometry only."""

import math
import xml.etree.ElementTree as ET

import numpy as np
from arm_collision_bounds import intersects
from arm_kinematics import rotation_error
from cartesian_path import interpolate_joints
from cooperative_transfer import rigid_pose
from fixture_collision_bounds import sdf_pose


class CarriedConnector:
    def __init__(self, world, plug):
        link = ET.parse(world).find(f"world/model[@name='wire_harness_multi']/link[@name='{plug}']")
        if link is None:
            raise ValueError("Known connector collision geometry required")
        self.shapes = []
        for collision in link.findall("collision"):
            geometry = collision.find("geometry")
            name = collision.get("name", "")
            if geometry.find("box") is not None and name.startswith("housing"):
                half = np.array(list(map(float, geometry.findtext("box/size").split()))) / 2
                role = "housing"
            elif geometry.find("cylinder") is not None and name.startswith("pin_"):
                radius = float(geometry.findtext("cylinder/radius"))
                half = np.array([radius, radius, float(geometry.findtext("cylinder/length")) / 2])
                role = "pin"
            else:
                raise ValueError("Unknown carried connector collision shape")
            if half.shape != (3,) or not np.isfinite(half).all() or min(half) <= 0:
                raise ValueError("Finite positive intrinsic dimensions required")
            # Deliberately ignore model pose and link pose: they are scene
            # object placement, not perception. Collision poses are intrinsic.
            self.shapes.append((name, role, sdf_pose(collision), half))
        if [r for _, r, _, _ in self.shapes].count("housing") != 1 or [
            r for _, r, _, _ in self.shapes
        ].count("pin") != 2:
            raise ValueError("Exactly one housing and two physical pins required")

    def boxes(self, world_from_connector):
        body = rigid_pose(world_from_connector)
        return [
            (name, role, ((body @ local)[:3, 3], (body @ local)[:3, :3], half))
            for name, role, local, half in self.shapes
        ]

    def fixture_overlap(self, pose, fixtures, *, margin=0.0005, seating=None):
        if not math.isfinite(margin) or not 0 <= margin <= 0.001:
            raise ValueError("Carried-part margin must be within [0, 1] mm per box")
        body = rigid_pose(pose)
        allowed_prefix = None
        if seating is not None:
            socket = seating["socket_id"]
            if not isinstance(socket, str) or not socket.isidentifier():
                raise ValueError("Known socket identifier required")
            goal = rigid_pose(seating["connector_pose"])
            delta = np.linalg.inv(goal) @ body
            if (
                np.linalg.norm(delta[:2, 3]) < 0.002
                and -0.0001 <= delta[2, 3] <= 0.0031
                and np.linalg.norm(rotation_error(body[:3, :3], goal[:3, :3])) < 0.005
            ):
                allowed_prefix = "socket_" + socket + "_"
        intended = []
        for name, role, box in self.boxes(body):
            nearby = np.linalg.norm(
                fixtures.centers - box[0], axis=1
            ) <= fixtures.radii + np.linalg.norm(box[2] + margin)
            for index in np.flatnonzero(nearby):
                fixture, shape = fixtures.shapes[index]
                if intersects(box, shape, margin=margin):
                    contact = {"connector_shape": name, "fixture": fixture}
                    # Only the housing on the selected socket rim at the
                    # seating pose. Pins and every other fixture stay checked.
                    if role == "housing" and allowed_prefix and fixture.startswith(allowed_prefix):
                        intended.append(contact)
                    else:
                        return contact, intended
        return None, intended

    def screen(self, arm, path, connector_from_tool, fixtures, *, seating=None):
        path = np.asarray(path, dtype=float)
        if path.ndim != 2 or len(path) < 2 or not np.isfinite(path).all():
            raise ValueError("Finite joint path required")
        relation = np.linalg.inv(rigid_pose(connector_from_tool))
        count = max(
            81,
            1
            + math.ceil(
                sum(float(np.max(np.abs(b - a))) for a, b in zip(path[:-1], path[1:])) / 0.01
            ),
        )
        intended = 0
        for fraction in np.linspace(0, 1, count):
            pose = arm.forward(interpolate_joints(path, fraction)) @ relation
            overlap, contacts = self.fixture_overlap(pose, fixtures, seating=seating)
            intended += len(contacts)
            if overlap is not None:
                return {
                    "clear": False,
                    "fraction": float(fraction),
                    "overlap": overlap,
                    "samples": count,
                    "intended_housing_contact_samples": intended,
                }
        return {
            "clear": True,
            "samples": count,
            "margin_per_box_m": 0.0005,
            "intended_housing_contact_samples": intended,
            "scope": "Discrete enclosing-box check; cylinder boxes are conservative. No wire collision, actual contact, force or insertion success proof",
        }
