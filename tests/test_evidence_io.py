import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from evidence_io import write_json_atomic


class EvidenceIOTest(unittest.TestCase):
    def test_success_replaces_valid_json_and_removes_temporary_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "progress.json"
            write_json_atomic(path, {"state": "running"})
            write_json_atomic(path, {"state": "finished"})
            self.assertEqual(json.loads(path.read_text()), {"state": "finished"})
            self.assertEqual(list(Path(directory).iterdir()), [path])

    def test_invalid_payload_preserves_previous_record(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "progress.json"
            write_json_atomic(path, {"state": "running"})
            for value in (object(), float("nan")):
                with self.subTest(value=value), self.assertRaises((TypeError, ValueError)):
                    write_json_atomic(path, {"invalid": value})
                self.assertEqual(json.loads(path.read_text()), {"state": "running"})
