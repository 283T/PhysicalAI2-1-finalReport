import copy
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import numpy as np
from arm_collision_bounds import intersects
from assembly_case import make_case, validate_connection_plan
from build_case import configure_case_pallet, harness_model


def prediction(design):
    sockets = {row["id"]: row for row in design["sockets"]}
    return {key: design[key] for key in ("drawing_id", "board_id", "harness_id")} | {
        "connections": [
            {**row, **{key: sockets[row["socket"]][key] for key in ("x_mm", "y_mm", "yaw_deg")}}
            for row in design["connections"]
        ]
    }


class AssemblyCaseTest(unittest.TestCase):
    def test_pallet_retains_wire_support_area_and_clears_left_robot_base(self):
        pallet = ET.Element("link")
        configure_case_pallet(pallet)
        boxes = []
        for shape in pallet.findall("collision"):
            center = np.array([float(v) for v in shape.findtext("pose").split()[:3]])
            half = np.array([float(v) for v in shape.findtext("geometry/box/size").split()]) / 2
            boxes.append((center, np.eye(3), half))
        base = (np.array([0.09325, -0.287, 0.2165]), np.eye(3), np.array([0.16525, 0.115, 0.0965]))
        self.assertFalse(any(intersects(b, base, margin=0.005) for b in boxes))
        for point in ([-0.25, 0.17, 0.12], [-0.46, -0.02, 0.12], [-0.05, 0.28, 0.12]):
            self.assertTrue(
                any(np.all(np.abs(np.array(point) - b[0]) <= b[2] + 1e-9) for b in boxes)
            )

    def test_design_and_placement_are_independent(self):
        a, b = make_case(100, 1), make_case(100, 2)
        self.assertEqual(a["design"], b["design"])
        self.assertNotEqual(a["simulation_initialization"], b["simulation_initialization"])
        self.assertEqual(a, make_case(100, 1))
        self.assertNotEqual(a["design"], make_case(101, 1)["design"])

    def test_one_physical_harness_with_all_terminals_connected(self):
        world = ET.Element("world")
        model = harness_model(world, make_case(100, 1)["simulation_initialization"])
        links = {link.get("name") for link in model.findall("link")}
        reachable = {"junction"}
        joints = model.findall("joint")
        for _ in joints:
            for joint in joints:
                if joint.findtext("parent") in reachable:
                    reachable.add(joint.findtext("child"))
        self.assertEqual(links, reachable)
        self.assertEqual(len(joints), len(links) - 1)
        self.assertTrue({"P1", "P2", "P3"} <= links)
        self.assertEqual(len(model.findall("joint[@type='universal']")), 27)

    def test_every_generated_mapping_is_complete(self):
        for seed in range(100):
            design = make_case(seed, 1)["design"]
            self.assertEqual(len(validate_connection_plan(prediction(design), design)), 3)

    def test_partial_duplicate_wrong_board_or_coordinates_are_rejected(self):
        design = make_case(100, 1)["design"]
        good = prediction(design)
        bad = []
        partial = copy.deepcopy(good)
        partial["connections"].pop()
        bad.append(partial)
        duplicate = copy.deepcopy(good)
        duplicate["connections"][1] = duplicate["connections"][0]
        bad.append(duplicate)
        wrong = copy.deepcopy(good)
        wrong["board_id"] = "OTHER"
        bad.append(wrong)
        shifted = copy.deepcopy(good)
        shifted["connections"][0]["x_mm"] += 1
        bad.append(shifted)
        rotated = copy.deepcopy(good)
        rotated["connections"][0]["yaw_deg"] += 90
        bad.append(rotated)
        swapped = copy.deepcopy(good)
        swapped["connections"][0]["plug_id"], swapped["connections"][1]["plug_id"] = (
            swapped["connections"][1]["plug_id"],
            swapped["connections"][0]["plug_id"],
        )
        bad.append(swapped)
        for plan in bad:
            with self.assertRaises(ValueError):
                validate_connection_plan(plan, design)


if __name__ == "__main__":
    unittest.main()
