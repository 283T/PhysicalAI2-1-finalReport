import math
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from refine_physics_timestep import refine


class PhysicsRefinementTests(unittest.TestCase):
    def test_refinement_preserves_every_other_element_and_input(self):
        with tempfile.TemporaryDirectory() as directory:
            case = Path(directory) / "source"
            case.mkdir()
            original = b'<sdf><world><physics><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics><model name="wire"><link name="flexible"/></model></world></sdf>'
            (case / "world.sdf").write_bytes(original)
            (case / "drawing.png").write_bytes(b"drawing")
            output = Path(directory) / "refined"
            refine(case, output, 0.0001)
            actual = ET.parse(output / "world.sdf").getroot()
            self.assertEqual(actual.findtext("world/physics/max_step_size"), "0.0001")
            actual.find("world/physics/max_step_size").text = "0.001"
            for element in actual.iter():
                element.tail = None
                if element.text is not None and not element.text.strip():
                    element.text = None
            self.assertEqual(ET.tostring(actual), ET.tostring(ET.fromstring(original)))
            self.assertEqual((case / "world.sdf").read_bytes(), original)
            self.assertEqual((output / "drawing.png").read_bytes(), b"drawing")
            with self.assertRaises(ValueError):
                refine(case, output, 0.00005)

    def test_coarser_nonfinite_or_ambiguous_steps_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            case = Path(directory) / "source"
            case.mkdir()
            world = case / "world.sdf"
            world.write_text(
                "<sdf><world><physics><max_step_size>0.001</max_step_size></physics></world></sdf>"
            )
            for value in (0, -1, 0.001, 0.002, math.nan, math.inf):
                with self.subTest(value=value), self.assertRaises(ValueError):
                    refine(case, Path(directory) / "out", value)
            world.write_text("<sdf><world><physics/></world></sdf>")
            with self.assertRaises(ValueError):
                refine(case, Path(directory) / "out", 0.0001)
