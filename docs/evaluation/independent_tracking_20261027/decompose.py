"""Kinematic decomposition of the failed trial's ordered relative-tool residual."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts"))
from evidence_io import write_json_atomic
from feedback_pacing import pose_from_feedback
from plan_reach import load_arms


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("world", "models", "pickup", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Preserve previous diagnostic")
    report = json.loads((args.pickup / "validation.json").read_text())
    if report.get("trial_state") != "finished":
        parser.error("Finished trial required")
    state = report["stop_diagnostics"]["last_observation"]["arms"]
    plan = json.loads((args.pickup / "nest_plan.json").read_text())
    phase_index = int(report["phase"].removeprefix("nest_transfer_"))
    order = list(plan["attempts"][plan["selected_attempt"]]["phases"][phase_index]["joint_paths"])
    left, right = order
    arms = load_arms(args.world, args.models)
    commands = {n: np.array(state[n]["command_q_rad"]) for n in order}
    targets, jacobians = {}, {}
    for name in order:
        targets[name], jacobians[name] = arms[name].forward(commands[name], jacobian=True)

    def relative(q):
        return np.linalg.inv(arms[left].forward(q[left])) @ arms[right].forward(q[right])

    target = relative(commands)
    observed = np.linalg.inv(pose_from_feedback(state[left])) @ pose_from_feedback(state[right])
    measured = relative({n: state[n]["q_rad"] for n in order})
    delta = targets[right][:3, 3] - targets[left][:3, 3]
    rotation = targets[left][:3, :3].T
    contributions, derivative_errors = [], []
    for name in order:
        error = np.array(state[name]["q_rad"]) - commands[name]
        for joint in range(len(error)):
            jac = jacobians[name]
            derivative = rotation @ (
                -jac[:3, joint] - np.cross(jac[3:, joint], delta)
            ) if name == left else rotation @ jac[:3, joint]
            step = 1e-6
            plus = {n: q.copy() for n, q in commands.items()}
            minus = {n: q.copy() for n, q in commands.items()}
            plus[name][joint] += step
            minus[name][joint] -= step
            numeric = (relative(plus)[:3, 3] - relative(minus)[:3, 3]) / (2 * step)
            derivative_errors.append(float(np.linalg.norm(numeric - derivative)))
            vector = derivative * error[joint]
            contributions.append({"arm": name, "joint_index_1based": joint + 1,
                                  "joint_error_rad": float(error[joint]),
                                  "relative_translation_contribution_m": vector.tolist(),
                                  "norm_m": float(np.linalg.norm(vector))})
    linear = np.sum([c["relative_translation_contribution_m"] for c in contributions], axis=0)
    result = {
        "scope": "First-order kinematic attribution, not torque, friction, contact-force measurement or causal proof. Vector contributions can cancel; their norms must not be added as a scalar error.",
        "source_trial_success": report["success"], "tool_order": order,
        "sim_time_s": report["stop_diagnostics"]["sim_time_s"],
        "native_relative_translation_residual_m": (observed[:3, 3] - target[:3, 3]).tolist(),
        "fk_relative_translation_residual_m": (measured[:3, 3] - target[:3, 3]).tolist(),
        "linear_relative_translation_residual_m": linear.tolist(),
        "linearization_error_m": float(np.linalg.norm(linear - (measured[:3, 3] - target[:3, 3]))),
        "native_fk_relative_discrepancy_m": float(np.linalg.norm(observed[:3, 3] - measured[:3, 3])),
        "maximum_central_difference_derivative_error_m_per_rad": max(derivative_errors),
        "ranked_contributions": sorted(contributions, key=lambda c: c["norm_m"], reverse=True),
        "input_sha256": {"validation": hashlib.sha256((args.pickup / "validation.json").read_bytes()).hexdigest(),
                         "world": hashlib.sha256(args.world.read_bytes()).hexdigest(),
                         "source": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},
    }
    if max(derivative_errors) > 1e-7:
        raise RuntimeError("Analytic derivative differs from central difference")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_json_atomic(args.output, result)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
