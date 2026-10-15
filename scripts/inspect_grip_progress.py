"""Read complete records from a possibly unfinished gzip grip log; no robot control."""

import argparse
import json
import math
from pathlib import Path

from evidence_io import write_json_atomic
from evidence_stream import open_evidence


def command_fraction(path, command):
    best = (float("inf"), 0.0)
    for index, (a, b) in enumerate(zip(path, path[1:])):
        delta = [y - x for x, y in zip(a, b)]
        denominator = sum(x * x for x in delta)
        blend = (
            max(0, min(1, sum((x - y) * z for x, y, z in zip(command, a, delta)) / denominator))
            if denominator
            else 0
        )
        error = sum((x - y - blend * z) ** 2 for x, y, z in zip(command, a, delta))
        best = min(best, (error, (index + blend) / (len(path) - 1)))
    return best[1]


def inspect(pickup):
    first, latest, partial = {}, {}, False
    with open_evidence(pickup / "grip_observations.jsonl.gz") as stream:
        try:
            for line in stream:
                row = json.loads(line)
                first.setdefault(row["phase"], row["sim_time_s"])
                latest[row["grip"]] = row
        except (EOFError, json.JSONDecodeError):
            partial = True
    result = {
        "scope": "Diagnostic snapshot only; verify process liveness separately. Commanded path fraction is not physical task completion",
        "incomplete_tail": partial,
        "grips": {},
    }
    for name, row in latest.items():
        state = row["observed"]
        pose, rotation = state["command_tcp_pose"], state["tcp_rotation_world"]
        trace = sum(sum(rotation[k][i] * pose[k][i] for k in range(3)) for i in range(3))
        result["grips"][name] = {
            "phase": row["phase"],
            "sim_time_s": row["sim_time_s"],
            "phase_elapsed_s": row["sim_time_s"] - first[row["phase"]],
            "grip_check": row["check"],
            "tracking_position_m": math.sqrt(
                sum((x - pose[i][3]) ** 2 for i, x in enumerate(state["tcp"]))
            ),
            "tracking_angle_rad": math.acos(max(-1, min(1, (trace - 1) / 2))),
            "joint_errors_rad": [a - b for a, b in zip(state["q_rad"], state["command_q_rad"])],
        }
    if "primary" in latest and latest["primary"]["phase"].startswith("nest_transfer_"):
        row = latest["primary"]
        plan = json.loads((pickup / "nest_plan.json").read_text())
        index = int(row["phase"].rsplit("_", 1)[1])
        path = plan["attempts"][plan["selected_attempt"]]["phases"][index]["joint_paths"][
            "right_arm"
        ]
        result["commanded_phase_fraction"] = command_fraction(
            path, row["observed"]["command_q_rad"]
        )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pickup", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = inspect(args.pickup)
    if args.output:
        write_json_atomic(args.output, result)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
