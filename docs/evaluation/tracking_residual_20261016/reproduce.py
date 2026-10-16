"""Decompose the failed trial's native tool residual using the robot Jacobian."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, "scripts")
from plan_reach import load_arms

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output", required=True, type=Path)
args = parser.parse_args()
if args.output.exists():
    raise ValueError("Preserve previous diagnostic")
root = Path("docs/evaluation/cooperative_nest_20261016/stepped_descent_tracking_failure")
source = root / "pickup/validation.json"
report = json.loads(source.read_text())
arms = load_arms(root / "case/world.sdf", root / "models")
result = {
    "scope": "Offline kinematic decomposition of native robot feedback. Linear terms are approximate; they do not identify applied torque, integrator state or the physical cause of the residual. No actuation or success claim.",
    "trial_success": report["success"],
    "simulation_time_s": report["stop_diagnostics"]["sim_time_s"],
    "arms": {},
}
for name, state in report["stop_diagnostics"]["last_observation"]["arms"].items():
    commanded = np.array(state["command_q_rad"])
    measured = np.array(state["q_rad"])
    pose, jacobian = arms[name].forward(commanded, jacobian=True)
    observed = arms[name].forward(measured)
    contributions = (jacobian[:3] * (measured - commanded)).T
    native_delta = np.array(state["tcp"]) - pose[:3, 3]
    fk_delta = observed[:3, 3] - pose[:3, 3]
    result["arms"][name] = {
        "joint_error_rad": (measured - commanded).tolist(),
        "linearized_translation_by_joint_m": contributions.tolist(),
        "exact_fk_translation_residual_m": fk_delta.tolist(),
        "native_translation_residual_m": native_delta.tolist(),
        "linearization_error_m": float(np.linalg.norm(contributions.sum(axis=0) - fk_delta)),
        "native_fk_discrepancy_m": float(np.linalg.norm(native_delta - fk_delta)),
    }
inputs = [
    source,
    root / "case/world.sdf",
    *sorted((root / "models").glob("*.urdf")),
    Path("scripts/arm_kinematics.py"),
    Path("scripts/plan_reach.py"),
    Path(__file__),
]
result["input_sha256"] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result["arms"], indent=2))
