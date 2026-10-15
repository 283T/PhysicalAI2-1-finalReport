import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, "scripts")
from arm_collision_bounds import ArmCollisionBounds, FingerStateBounds
from plan_reach import load_arms
from regrasp_plan import plan_top_pickup
from stepped_wire_profile import stepped_profile

parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
root = args.output
if root.exists() and any(root.iterdir()):
    raise ValueError("Preserve previous trials; use a new directory")
root.mkdir(parents=True, exist_ok=True)
original = Path("docs/evaluation/dual_pickup_20261016/support140_settle30_dual_hold/models")
world = Path(
    "docs/evaluation/cooperative_nest_20261016/left_wrist640_pacing_stalled/case/world.sdf"
)
for name in ["left_arm", "right_arm"]:
    robot = ET.parse(original / (name + ".urdf")).getroot()
    if name == "left_arm":
        robot = stepped_profile(robot, "left_arm_")
    ET.ElementTree(robot).write(root / (name + ".urdf"), encoding="utf-8", xml_declaration=True)
arms = load_arms(world, root)
bounds = {
    n: FingerStateBounds(
        ArmCollisionBounds(root / (n + ".urdf"), a),
        {n + "_finger_" + side + "_joint": [0.0175, 0.0185] for side in ["left", "right"]},
    )
    for n, a in arms.items()
}
body = json.load(open("docs/evaluation/cooperative_nest_20261016/top_pickup_closure_checked.json"))[
    "connector_pose"
]
report = plan_top_pickup(arms, bounds, world, {n: np.zeros(6) for n in arms}, "left_arm", body)
report["scope_note"] = (
    "Offline candidate from zero joints with right arm assumed parked and connector at nominal nest seat; no actual seat, parking, camera, grip, insertion or support-wire success. Gripper shape is permanent and would require a newly built complete world before any physical experiment."
)
report["input_sha256"] = {
    str(p): hashlib.sha256(p.read_bytes()).hexdigest()
    for p in [
        world,
        original / "left_arm.urdf",
        original / "right_arm.urdf",
        Path("docs/evaluation/cooperative_nest_20261016/top_pickup_closure_checked.json"),
    ]
}
(root / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
print("top pickup geometry:", report["success"])
print(
    [
        {k: v for k, v in row.items() if k in ["success", "error", "closure", "fixtures"]}
        for row in report["attempts"]
    ]
)

(root / "manifest.json").write_text(
    json.dumps(
        {
            str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in root.iterdir()
            if p.is_file() and p.name != "manifest.json"
        },
        indent=2,
    )
    + "\n"
)
