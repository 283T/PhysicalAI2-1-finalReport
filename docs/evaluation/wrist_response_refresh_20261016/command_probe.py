"""Passive measurement only: observe native controller refresh, never publish."""

import argparse
import hashlib
import json
import os
import threading
import time
from pathlib import Path

from gz.msgs10.double_pb2 import Double
from gz.transport13 import Node

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--partition")
parser.add_argument("--topic", default="/left_arm/joint_0/cmd")
parser.add_argument("--duration", type=float, default=3)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
if args.output.exists() or not 1 <= args.duration <= 10:
    raise ValueError("New output and duration 1..10 seconds required")
if args.partition:
    os.environ["GZ_PARTITION"] = args.partition
node = Node()
rows = []
ready = threading.Event()


def receive(message):
    rows.append((time.monotonic(), message.data))
    ready.set()


if not node.subscribe(Double, args.topic, receive):
    raise RuntimeError("Subscription failed")
ready.wait(5)
start = time.monotonic()
time.sleep(args.duration)
end = time.monotonic()
node.unsubscribe(args.topic)
samples = [(t, v) for t, v in rows if start <= t <= end]
report = {
    "scope": "Passive native command receipt only; no commands sent and no assembly success evidence",
    "partition": os.environ.get("GZ_PARTITION"),
    "topic": args.topic,
    "window_wall_s": end - start,
    "received_messages": len(samples),
    "values": sorted(set(v for _, v in samples)),
    "arrival_intervals_s": [b[0] - a[0] for a, b in zip(samples, samples[1:])],
    "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(report, indent=2) + "\n")
print(report)
