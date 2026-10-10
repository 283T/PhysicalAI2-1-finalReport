import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from clean_generated import clean


class CleanupTest(unittest.TestCase):
    def test_preview_and_apply_preserve_models_current_scene_and_source(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            obsolete = root / "outputs/perception_eval_seed2"
            obsolete.mkdir(parents=True)
            (obsolete / "stale.json").write_text("{}")
            protected = (
                "outputs/perception_demo_seed2/world.sdf",
                "outputs/melfa_install/setup.bash",
                ".cache/ollama/model",
                "recordings/demo.mp4",
                "scripts/source.py",
            )
            for name in protected:
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("keep")
            clean(root)
            self.assertTrue(obsolete.exists())
            clean(root, apply=True)
            self.assertFalse(obsolete.exists())
            self.assertTrue(all((root / name).exists() for name in protected))

    def test_symlink_is_removed_without_touching_external_target(self):
        with tempfile.TemporaryDirectory() as folder, tempfile.TemporaryDirectory() as outside:
            root = Path(folder)
            external = Path(outside) / "keep"
            external.write_text("keep")
            (root / "log").symlink_to(outside, target_is_directory=True)
            clean(root, apply=True)
            self.assertTrue(external.exists())
            self.assertFalse((root / "log").is_symlink())
