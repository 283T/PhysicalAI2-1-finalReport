"""Atomic JSON evidence writes: invalid data must not destroy the last valid record."""

import json
from pathlib import Path


def write_json_atomic(path, value):
    path = Path(path)
    payload = json.dumps(value, indent=2, allow_nan=False) + "\n"
    temporary = path.with_name("." + path.name + ".tmp")
    temporary.write_text(payload)
    temporary.replace(path)
