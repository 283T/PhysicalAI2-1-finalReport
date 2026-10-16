"""Hold the measured finger midpoint with a bounded, decaying squeeze offset."""

import hashlib
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from numbers import Real

from gripper_model import MAX_TRAVEL_M


@dataclass(frozen=True)
class CentredGrip:
    reference: tuple
    start_sim_time_s: float
    squeeze_m: float
    p_gain: float
    i_gain: float

    def __post_init__(self):
        object.__setattr__(self, "reference", tuple(float(x) for x in self.reference))
        if (
            len(self.reference) != 2
            or not all(
                math.isfinite(x)
                for x in (
                    *self.reference,
                    self.start_sim_time_s,
                    self.squeeze_m,
                    self.p_gain,
                    self.i_gain,
                )
            )
            or self.start_sim_time_s < 0
            or self.p_gain <= 0
            or self.i_gain < 0
            or not 0 < self.squeeze_m <= min(self.reference) - 0.0002
            or max(self.reference) > MAX_TRAVEL_M
        ):
            raise ValueError(
                "Finite measured fingers, positive squeeze and supported force-PID gains required"
            )

    def commands(self, stamp):
        if stamp is None or not math.isfinite(stamp) or stamp < self.start_sim_time_s:
            raise ValueError("Physical clock must not precede centred-hold start")
        offset = self.squeeze_m * math.exp(
            -self.i_gain / self.p_gain * (stamp - self.start_sim_time_s)
        )
        return tuple(float(q - offset) for q in self.reference)

    def describe(self):
        return {
            "reference_fingers_m": list(self.reference),
            "start_sim_time_s": self.start_sim_time_s,
            "initial_squeeze_m": self.squeeze_m,
            "p_gain": self.p_gain,
            "i_gain": self.i_gain,
            "initial_targets_m": list(self.commands(self.start_sim_time_s)),
            "scope": "Robot finger targets only. Equal offsets preserve the commanded midpoint. Exponential decay bounds the ideal blocked-joint PI output; no measured-force or physical holding claim.",
        }


def finger_commands(value, stamp):
    if isinstance(value, CentredGrip):
        return value.commands(stamp)
    values = (value, value) if isinstance(value, Real) else tuple(value)
    if len(values) != 2 or not all(math.isfinite(x) and 0 <= x <= MAX_TRAVEL_M for x in values):
        raise ValueError("Two finite mechanical finger command values required")
    return tuple(float(x) for x in values)


def finger_pid(world, arm):
    model = ET.parse(world).getroot().find(f"world/model[@name='{arm}']")
    if model is None:
        raise ValueError("Known arm model required")
    gains = []
    for side in ("left", "right"):
        matches = [
            p
            for p in model.findall("plugin")
            if p.get("name") == "gz::sim::systems::JointPositionController"
            and p.findtext("joint_name") == f"{arm}_finger_{side}_joint"
        ]
        if len(matches) != 1 or matches[0].findtext("use_velocity_commands", "false") != "false":
            raise ValueError("One force-PID controller per measured finger required")
        params = {
            k: float(matches[0].findtext(k, "nan"))
            for k in ("p_gain", "i_gain", "d_gain", "i_max", "i_min", "cmd_max", "cmd_min")
        }
        if (
            not all(math.isfinite(x) for x in params.values())
            or params["p_gain"] <= 0
            or params["i_gain"] < 0
        ):
            raise ValueError("Finite positive-P/nonnegative-I parameters required")
        gains.append(params)
    if gains[0] != gains[1]:
        raise ValueError("Matched finger PID parameters required for this holding model")
    return gains[0]


def activate(session, args, report, checkpoint, arm, prefix, guard, requested_squeeze_m, phase):
    if not getattr(args, "centred_grips", False):
        return
    if (
        guard not in session.evaluation_guards
        or prefix not in report.get("grip_references", {})
        or prefix + "_guard_failure" in report
        or prefix in report.get("centred_grip_plans", {})
        or isinstance(session.grips[arm], CentredGrip)
    ):
        raise RuntimeError(
            "One new centred holding episode with active original grip monitoring required"
        )
    digest = hashlib.sha256(args.world.read_bytes()).hexdigest()
    if digest != report["world_sha256"]:
        raise RuntimeError("World changed before selecting finger holding parameters")
    row = session.snapshot()
    if row is None:
        raise RuntimeError("Fresh robot feedback required for centred holding")
    reference = tuple(float(x) for x in row["arms"][arm]["fingers"])
    squeeze = min(requested_squeeze_m, min(reference) - 0.0002)
    pid = finger_pid(args.world, arm)
    plan = CentredGrip(reference, row["sim_time_s"], squeeze, pid["p_gain"], pid["i_gain"])
    session.grips[arm] = plan
    report.setdefault("centred_grip_plans", {})[prefix] = {
        **plan.describe(),
        "arm": arm,
        "world_sha256": digest,
        "pid_parameters": pid,
        "requested_squeeze_m": requested_squeeze_m,
    }
    checkpoint(phase)
    session.dwell(phase, 1.0)
