"""Isolate gzip writing cost on unchanged recorded JSONL; no simulator control."""

import argparse
import gzip
import hashlib
import io
import json
import platform
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, "scripts")
from evidence_io import write_json_atomic
from evidence_stream import evidence_digest, open_evidence

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--pickup", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--records", type=int, default=5000)
args = parser.parse_args()
if args.output.exists() or args.records <= 0:
    raise ValueError("New output and positive sample count required")
path = args.pickup / "grip_observations.jsonl.gz"
with open_evidence(path) as stream:
    lines = []
    for line in stream:
        json.loads(line)
        lines.append(line)
        if len(lines) == args.records:
            break
raw = "".join(lines).encode()
rows = []
for repetition, levels in enumerate(((9, 1, 6), (1, 6, 9), (6, 9, 1))):
    for level in levels:
        target = io.BytesIO()
        wall, cpu = time.perf_counter(), time.process_time()
        with gzip.open(target, "wt", compresslevel=level) as stream:
            for line in lines:
                stream.write(line)
        elapsed_cpu, elapsed_wall = time.process_time() - cpu, time.perf_counter() - wall
        compressed = target.getvalue()
        if gzip.decompress(compressed) != raw:
            raise RuntimeError("Lossless evidence check failed")
        rows.append(
            {
                "repetition": repetition,
                "level": level,
                "cpu_s": elapsed_cpu,
                "wall_s": elapsed_wall,
                "compressed_bytes": len(compressed),
                "decompressed_bytes_identical": True,
            }
        )
result = {
    "scope": "Offline prefix-only compression benchmark. Same JSONL observations and values, no decimation. Does not measure physical real-time factor or establish a simulator bottleneck.",
    "python": platform.python_version(),
    "machine": platform.machine(),
    "source_compressed_sha256": evidence_digest(path),
    "sample_records": len(lines),
    "sample_bytes": len(raw),
    "sample_sha256": hashlib.sha256(raw).hexdigest(),
    "measurements": rows,
    "summary": {
        str(level): {
            "median_cpu_s": statistics.median(
                row["cpu_s"] for row in rows if row["level"] == level
            ),
            "median_wall_s": statistics.median(
                row["wall_s"] for row in rows if row["level"] == level
            ),
            "compressed_bytes": next(
                row["compressed_bytes"] for row in rows if row["level"] == level
            ),
        }
        for level in (1, 6, 9)
    },
}
write_json_atomic(args.output, result)
print(json.dumps({k: result[k] for k in ("sample_records", "sample_bytes", "summary")}, indent=2))
