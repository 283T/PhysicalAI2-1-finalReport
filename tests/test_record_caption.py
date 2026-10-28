import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from record_dual_hold import update_caption


class RecordCaptionTest(unittest.TestCase):
    def test_updates_preserve_inode_and_never_truncate_the_reload_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "caption.txt"
            path.write_bytes(b" " * 255 + b"\n")
            inode = path.stat().st_ino
            for i in range(100):
                update_caption(path, f"phase=dual_hold | elapsed={i}.0s")
                self.assertEqual(path.stat().st_ino, inode)
                self.assertEqual(len(path.read_bytes()), 256)
                self.assertTrue(path.read_bytes().startswith(b"phase=dual_hold"))

    def test_invalid_update_does_not_destroy_existing_caption(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "caption.txt"
            original = b" " * 255 + b"\n"
            path.write_bytes(original)
            with self.assertRaises(ValueError):
                update_caption(path, "x" * 256)
            self.assertEqual(path.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
