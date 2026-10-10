import io
import json
import sys
import tempfile
import unittest
import urllib.error
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from local_vision import infer_drawing


class LocalVisionTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.image = Path(self.directory.name) / "drawing.png"
        self.image.write_bytes(b"synthetic image")
        self.output = self.image.with_name("plan.json")
        self.output.write_text("stale accepted plan")
        self.audit = self.output.with_suffix(".audit.json")
        self.audit.write_text("stale audit")

    def test_success_uses_loopback_and_preserves_raw_audit_before_validation(self):
        raw = {"done": True, "message": {"content": '{"connections": []}'}}
        with patch(
            "local_vision.urllib.request.urlopen", return_value=io.BytesIO(json.dumps(raw).encode())
        ) as send:
            extracted, audit = infer_drawing(
                self.image, self.output, "read rows", {"type": "object"}, num_predict=768
            )
        request = send.call_args[0][0]
        self.assertEqual(request.full_url, "http://127.0.0.1:11434/api/chat")
        payload = json.loads(request.data)
        self.assertEqual(payload["options"]["num_predict"], 768)
        self.assertEqual(extracted, {"connections": []})
        self.assertEqual(json.loads(self.audit.read_text()), audit)
        self.assertFalse(self.output.exists())  # Caller must validate before accepting.

    def test_truncated_or_malformed_responses_leave_no_accepted_plan(self):
        for raw in (
            {"done": True, "done_reason": "length"},
            {"done": True, "message": {"content": "not JSON"}},
        ):
            with patch(
                "local_vision.urllib.request.urlopen",
                return_value=io.BytesIO(json.dumps(raw).encode()),
            ):
                with self.assertRaises(ValueError):
                    infer_drawing(self.image, self.output, "", {})
            self.assertFalse(self.output.exists())
            self.assertEqual(json.loads(self.audit.read_text())["response"], raw)

    def test_connection_failure_clears_stale_plan_and_audit(self):
        with patch(
            "local_vision.urllib.request.urlopen", side_effect=urllib.error.URLError("offline")
        ):
            with self.assertRaisesRegex(RuntimeError, "Local Ollama request failed"):
                infer_drawing(self.image, self.output, "", {})
        self.assertFalse(self.output.exists())
        self.assertFalse(self.audit.exists())
