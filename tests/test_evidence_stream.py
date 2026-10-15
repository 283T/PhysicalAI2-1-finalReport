import gzip
import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from evidence_stream import evidence_digest, open_evidence, split_evidence


class EvidenceStreamTests(unittest.TestCase):
    def fixture(self, directory):
        path = Path(directory) / "observations.jsonl.gz"
        text = "".join(json.dumps({"time": i, "value": i * i}) + "\n" for i in range(200))
        path.write_bytes(gzip.compress(text.encode(), mtime=0))
        return path, text

    def test_split_reassembles_exact_compressed_bytes_and_all_records(self):
        with tempfile.TemporaryDirectory() as directory:
            path, text = self.fixture(directory)
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            self.assertTrue(split_evidence(path, threshold=64, chunk_size=32))
            self.assertFalse(path.exists())
            self.assertEqual(evidence_digest(path), digest)
            with open_evidence(path) as stream:
                self.assertEqual(stream.read(), text)

    def test_small_or_live_unsplit_logs_keep_the_original_interface(self):
        with tempfile.TemporaryDirectory() as directory:
            path, text = self.fixture(directory)
            original = path.read_bytes()
            self.assertFalse(split_evidence(path))
            self.assertEqual(path.read_bytes(), original)
            with open_evidence(path) as stream:
                self.assertEqual(stream.read(), text)

    def test_tampering_order_traversal_and_ambiguity_rejected_before_reading(self):
        for corruption in ("bytes", "order", "traversal", "ambiguous", "total"):
            with self.subTest(corruption=corruption), tempfile.TemporaryDirectory() as directory:
                path, text = self.fixture(directory)
                original = path.read_bytes()
                split_evidence(path, threshold=64, chunk_size=32)
                manifest = path.with_name(path.name + ".parts.json")
                data = json.loads(manifest.read_text())
                if corruption == "bytes":
                    (path.parent / data["parts"][0]["file"]).write_bytes(b"tampered")
                elif corruption == "order":
                    data["parts"].reverse()
                elif corruption == "traversal":
                    data["parts"][0]["file"] = "../secret"
                elif corruption == "ambiguous":
                    path.write_bytes(original)
                else:
                    data["sha256"] = "0" * 64
                manifest.write_text(json.dumps(data))
                with self.assertRaises(ValueError):
                    with open_evidence(path):
                        self.fail("Unverified evidence must not be exposed")
