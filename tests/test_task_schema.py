import copy
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from task_schema import validate_task


class TaskValidation(unittest.TestCase):
    def setUp(self):
        self.board = json.loads((ROOT / "configs/board.json").read_text())
        self.task = json.loads((ROOT / "examples/task_a.json").read_text())

    def test_both_variants_are_registered(self):
        for variant, socket in (("a", "J1"), ("b", "J2")):
            task = json.loads((ROOT / f"examples/task_{variant}.json").read_text())
            self.assertEqual(validate_task(task, self.board)["id"], socket)

    def test_wrong_socket_label_is_rejected(self):
        self.task["socket"] = "J2"
        with self.assertRaises(ValueError):
            validate_task(self.task, self.board)

    def test_unknown_position_is_rejected(self):
        self.task["x_mm"] += 15
        with self.assertRaises(ValueError):
            validate_task(self.task, self.board)

    def test_bad_model_coordinates_are_rejected(self):
        for value in (True, float("nan"), float("inf"), "60", None):
            with self.subTest(value=value):
                task = copy.deepcopy(self.task)
                task["x_mm"] = value
                with self.assertRaises(ValueError):
                    validate_task(task, self.board)

    def test_units_mistake_is_rejected(self):
        self.task["x_mm"] = 0.06
        with self.assertRaises(ValueError):
            validate_task(self.task, self.board)

    def test_wrong_orientation_is_rejected(self):
        self.task["yaw_deg"] = 90
        with self.assertRaises(ValueError):
            validate_task(self.task, self.board)

    def test_wrong_calibration_is_rejected(self):
        self.task["board_origin_m"][0] += 0.01
        with self.assertRaises(ValueError):
            validate_task(self.task, self.board)


if __name__ == "__main__":
    unittest.main()
