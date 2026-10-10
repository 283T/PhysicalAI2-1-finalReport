import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from fixture_collision_bounds import FixtureBounds


class MovingBox:
    def boxes(self, q):
        return [("tool", (np.array([q[0], 0, 0]), np.eye(3), np.array([0.02] * 3)))]


class FixtureTests(unittest.TestCase):
    def fixture(self, source):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "world.sdf"
        path.write_text(source)
        return FixtureBounds(path)

    def test_collision_uses_model_link_and_shape_transforms(self):
        fixture = self.fixture(
            '<sdf><world><model name="fixtures"><static>true</static><pose>0.2 0 0 0 0 0</pose><link name="base"><pose>0.1 0 0 0 0 0</pose><collision name="board"><pose>0.1 0 0 0 0 0</pose><geometry><box><size>0.1 0.1 0.1</size></box></geometry></collision></link></model></world></sdf>'
        )
        np.testing.assert_allclose(fixture.centers, [[0.4, 0, 0]])
        result = fixture.screen({"a": MovingBox()}, {"a": np.array([[0.1], [0.7]])})
        self.assertFalse(result["clear"])
        self.assertEqual(result["shapes"], ["a/tool", "board"])
        self.assertTrue(
            fixture.screen({"a": MovingBox()}, {"a": np.array([[0.1], [0.2]])})["clear"]
        )

    def test_reject_unhandled_reference_frame_and_shape(self):
        for geometry, pose in (
            ("<sphere><radius>1</radius></sphere>", ""),
            ("<box><size>1 1 1</size></box>", '<pose relative_to="other">0 0 0 0 0 0</pose>'),
        ):
            with self.subTest(geometry=geometry), self.assertRaises(ValueError):
                self.fixture(
                    f'<sdf><world><model name="fixtures"><static>true</static><link name="base"><collision name="wall">{pose}<geometry>{geometry}</geometry></collision></link></model></world></sdf>'
                )

    def test_explicit_small_margin_keeps_actual_gap_and_rejects_invalid_limits(self):
        fixture = self.fixture(
            '<sdf><world><model name="fixtures"><static>true</static><link name="base"><collision name="rim"><pose>0.4 0 0 0 0 0</pose><geometry><box><size>0.1 0.1 0.1</size></box></geometry></collision></link></model></world></sdf>'
        )
        paths = {"a": np.array([[0.473], [0.48]])}
        self.assertFalse(fixture.screen({"a": MovingBox()}, paths)["clear"])
        self.assertTrue(fixture.screen({"a": MovingBox()}, paths, margin=0.001)["clear"])
        for margin in (-1, float("nan"), 0.006):
            with self.subTest(margin=margin), self.assertRaises(ValueError):
                fixture.screen({"a": MovingBox()}, paths, margin=margin)


if __name__ == "__main__":
    unittest.main()
