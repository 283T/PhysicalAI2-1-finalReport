"""Lossless, hash-verified storage of large gzip evidence in Git-sized parts."""

import gzip
import hashlib
import json
import tempfile
from contextlib import contextmanager

from evidence_io import write_json_atomic


def split_evidence(path, *, threshold=48 * 1024**2, chunk_size=32 * 1024**2):
    if not 0 < chunk_size <= threshold:
        raise ValueError("Positive chunk size within split threshold required")
    if path.stat().st_size <= threshold:
        return False
    manifest = path.with_name(path.name + ".parts.json")
    if manifest.exists() or any(path.parent.glob(path.name + ".part[0-9]*")):
        raise ValueError("Preserve existing split evidence")
    digest, size, parts = hashlib.sha256(), 0, []
    with path.open("rb") as source:
        while data := source.read(chunk_size):
            target = path.with_name(path.name + f".part{len(parts):04d}")
            with target.open("xb") as stream:
                stream.write(data)
            digest.update(data)
            size += len(data)
            parts.append(
                {
                    "file": target.name,
                    "bytes": len(data),
                    "sha256": hashlib.sha256(data).hexdigest(),
                }
            )
    if (
        size != path.stat().st_size
        or digest.hexdigest() != hashlib.sha256(path.read_bytes()).hexdigest()
    ):
        raise RuntimeError("Evidence changed during splitting; original retained")
    write_json_atomic(
        manifest,
        {
            "format": "gzip-byte-parts-v1",
            "original_file": path.name,
            "bytes": size,
            "sha256": digest.hexdigest(),
            "parts": parts,
        },
    )
    # Only this disposable archive copy is removed, after exact byte verification.
    # The original runtime log remains untouched in its trial output directory.
    path.unlink()
    return True


def evidence_digest(path):
    if path.exists():
        return hashlib.sha256(path.read_bytes()).hexdigest()
    return json.loads(path.with_name(path.name + ".parts.json").read_text())["sha256"]


@contextmanager
def open_evidence(path):
    manifest = path.with_name(path.name + ".parts.json")
    if not manifest.exists():
        with gzip.open(path, "rt") as stream:
            yield stream
        return
    if path.exists():
        raise ValueError("Ambiguous monolithic and split evidence")
    data = json.loads(manifest.read_text())
    if (
        data.get("format") != "gzip-byte-parts-v1"
        or data.get("original_file") != path.name
        or not data.get("parts")
    ):
        raise ValueError("Unknown or empty evidence manifest")
    with tempfile.TemporaryFile() as combined:
        digest, total = hashlib.sha256(), 0
        for index, part in enumerate(data["parts"]):
            if part.get("file") != path.name + f".part{index:04d}":
                raise ValueError("Evidence part order or scoped filename is invalid")
            part_digest, size = hashlib.sha256(), 0
            with (path.parent / part["file"]).open("rb") as source:
                while block := source.read(1024 * 1024):
                    combined.write(block)
                    part_digest.update(block)
                    digest.update(block)
                    size += len(block)
            if size != part["bytes"] or part_digest.hexdigest() != part["sha256"]:
                raise ValueError("Evidence part hash or size mismatch")
            total += size
        if total != data["bytes"] or digest.hexdigest() != data["sha256"]:
            raise ValueError("Reconstructed evidence hash or size mismatch")
        combined.seek(0)
        with gzip.open(combined, "rt") as stream:
            yield stream
