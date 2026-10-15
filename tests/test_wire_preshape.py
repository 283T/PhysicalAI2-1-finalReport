import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from wire_preshape import assess_preshape, preshape_target


class WirePreshapeTests(unittest.TestCase):
    def test_underestimated_image_width_cannot_shrink_known_wire_clearance(self):
        target = preshape_target(0.006, 0.005)
        self.assertGreaterEqual(target["target_gap_m"] - 0.008, 0.010 - 1e-12)
        self.assertLess(target["target_travel_m"], 0.018)

    def test_matching_total_opening_cannot_hide_unbalanced_fingers(self):
        target = preshape_target(0.006, 0.008)
        self.assertTrue(assess_preshape([0.006, 0.006], 0.006, target)["success"])
        self.assertFalse(assess_preshape([0.0068, 0.0052], 0.006, target)["success"])
        self.assertFalse(assess_preshape([0.004, 0.004], 0.006, target)["success"])

    def test_jaws_that_cannot_grip_wire_and_invalid_width_are_rejected(self):
        for rest, width in [(0.016, 0.008), (0.008, 0.008), (0.006, float("nan")), (0.006, 0.02)]:
            with self.subTest(rest=rest, width=width), self.assertRaises(ValueError):
                preshape_target(rest, width)
