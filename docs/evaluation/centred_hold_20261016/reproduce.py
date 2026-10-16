"""Ideal blocked-finger PI calculation only; does not run or control Gazebo."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

sys.path.insert(0, "scripts")
from centred_grip import CentredGrip, finger_pid
from evidence_io import write_json_atomic

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
if args.output.exists():
    raise ValueError("Preserve previous model calculation")
source = Path("docs/evaluation/grip_centering_20261016/snapshot.json")
world = Path(
    "docs/evaluation/cooperative_nest_20261016/preshape_nest_seat_wall_limit/case/world.sdf"
)
reference = json.loads(source.read_text())["grips"]["primary"]["closure_finger_travel_m"]
pid = finger_pid(world, "right_arm")
plan = CentredGrip(tuple(reference), 0, 0.0015, pid["p_gain"], pid["i_gain"])
integral = 0
dt = 0.01
maximum_error = 0
maximum_midpoint_change = 0
samples = []
for index in range(15001):
    stamp = index * dt
    commands = plan.commands(stamp)
    error = reference[0] - commands[0]
    integral += error * dt
    output = pid["p_gain"] * error + pid["i_gain"] * integral
    expected = pid["p_gain"] * plan.squeeze_m
    maximum_error = max(maximum_error, abs(output - expected))
    midpoint = (commands[0] - commands[1]) / 2
    maximum_midpoint_change = max(
        maximum_midpoint_change, abs(midpoint - (reference[0] - reference[1]) / 2)
    )
    if index % 2500 == 0:
        samples.append(
            {
                "elapsed_simulation_time_s": stamp,
                "finger_targets_m": list(commands),
                "ideal_unclipped_pi_output": output,
                "command_midpoint_m": midpoint,
            }
        )
result = {
    "scope": "Offline ideal blocked-finger model. Constant measured joint positions, zero initial integrator, derivative and saturation omitted. No actual force, grip, placement or simulator performance evidence.",
    "physical_tested": False,
    "final_acceptance_eligible": False,
    "plan": plan.describe(),
    "integration_step_s": dt,
    "maximum_ideal_pi_output_error": maximum_error,
    "maximum_command_midpoint_change_m": maximum_midpoint_change,
    "samples": samples,
    "input_sha256": {
        str(p): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (source, world, Path("scripts/centred_grip.py"))
    },
}
args.output.parent.mkdir(parents=True, exist_ok=True)
write_json_atomic(args.output, result)
print(
    json.dumps(
        {
            k: result[k]
            for k in ("maximum_ideal_pi_output_error", "maximum_command_midpoint_change_m")
        },
        indent=2,
    )
)
