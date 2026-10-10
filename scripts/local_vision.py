"""Audited structured image inference through local Ollama only."""

import base64
import hashlib
import json
import time
import urllib.error
import urllib.request

MODEL = "qwen3-vl:2b-instruct"
ENDPOINT = "http://127.0.0.1:11434/api/chat"


def infer_drawing(image_path, output_path, prompt, schema, model=MODEL, *, num_predict=256):
    """Return unvalidated model data and its audit; never read the board catalog."""
    image = image_path.read_bytes()
    payload = {
        "model": model,
        "messages": [
            {"role": "user", "content": prompt, "images": [base64.b64encode(image).decode("ascii")]}
        ],
        "stream": False,
        "format": schema,
        "options": {"temperature": 0, "num_ctx": 4096, "num_predict": num_predict},
        "keep_alive": "10m",
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    # A failed new inference must not expose a previous validated result or audit.
    output_path.unlink(missing_ok=True)
    audit_path = output_path.with_suffix(".audit.json")
    audit_path.unlink(missing_ok=True)
    request = urllib.request.Request(
        ENDPOINT, data=json.dumps(payload).encode(), headers={"Content-Type": "application/json"}
    )
    started = time.monotonic()
    try:
        with urllib.request.urlopen(request, timeout=300) as response:
            raw = json.load(response)
    except urllib.error.URLError as error:
        raise RuntimeError(
            "Local Ollama request failed. Start ollama serve and pull the model."
        ) from error
    audit = {
        "model": model,
        "image_sha256": hashlib.sha256(image).hexdigest(),
        "elapsed_seconds": time.monotonic() - started,
        "response": raw,
    }
    audit_path.write_text(json.dumps(audit, indent=2) + "\n")
    if not raw.get("done") or raw.get("done_reason") == "length":
        raise ValueError("Model output is incomplete")
    return json.loads(raw["message"]["content"]), audit
