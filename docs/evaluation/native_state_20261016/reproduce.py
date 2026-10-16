"""Decode a saved read-only Gazebo state response; never request or control a world."""

import argparse
import gzip
import hashlib
import json
import math
import re
from pathlib import Path

from google.protobuf import text_format
from gz.msgs10.double_v_pb2 import Double_V
from gz.msgs10.serialized_map_pb2 import SerializedStepMap


def hash64(name):
    value = 0xCBF29CE484222325
    for byte in name.encode("ascii"):
        value = ((value ^ byte) * 0x100000001B3) & ((1 << 64) - 1)
    return value


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--state", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
if args.output.exists():
    raise ValueError("Preserve previous diagnostic")
raw = args.state.read_bytes()
data = gzip.decompress(raw) if args.state.suffix == ".gz" else raw
state = text_format.Parse(data.decode("utf-8"), SerializedStepMap())
kinds = {
    hash64("gz_sim_components." + name): name
    for name in ("Name", "JointPosition", "JointVelocity", "JointForceCmd")
}
joints = {}
for entity in state.state.entities.values():
    values = {kinds[c.type]: c.component for c in entity.components.values() if c.type in kinds}
    name = values.get("Name", b"").decode("utf-8")
    if not re.fullmatch(r"(left|right)_arm_(rv4frl_joint_[1-6]|finger_(left|right)_joint)", name):
        continue
    if name in joints:
        raise ValueError("Ambiguous joint identity")
    record = {"entity_id": entity.id, "force_or_torque_verified": False}
    for key in ("JointPosition", "JointVelocity"):
        parsed = list(Double_V.FromString(values[key]).data)
        if len(parsed) != 1 or not math.isfinite(parsed[0]):
            raise ValueError("One finite native joint position/velocity required")
        record[key] = parsed[0]
    record["joint_force_cmd_component_present"] = "JointForceCmd" in values
    record["joint_force_cmd_serialized_bytes"] = len(values.get("JointForceCmd", b""))
    joints[name] = record
if len(joints) != 16:
    raise ValueError("All twelve arm joints and four finger joints required")
result = {
    "scope": "Read-only native state snapshot, not an action target or task-success result. Empty command serialization does not mean zero force/torque; saturation and applied effort are unproven.",
    "simulation_time_s": state.stats.sim_time.sec + state.stats.sim_time.nsec / 1e9,
    "component_type_ids": {v: k for k, v in kinds.items()},
    "joints": joints,
    "input_sha256": {
        str(args.state): hashlib.sha256(raw).hexdigest(),
        "uncompressed_state": hashlib.sha256(data).hexdigest(),
        "decoder": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    },
    "final_acceptance_eligible": False,
}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(result, indent=2) + "\n")
print(
    json.dumps(
        {
            "simulation_time_s": result["simulation_time_s"],
            "joints": len(joints),
            "empty_force_command_values": sum(
                not x["joint_force_cmd_serialized_bytes"] for x in joints.values()
            ),
        }
    )
)
