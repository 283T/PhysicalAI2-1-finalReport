"""Contact sensors and collision classification for the factory scene."""

import copy
import math
import re

from build_world import child, plugin

CONTACT_TOPIC = "/factory/arm_contacts"


def inactive_payload_arm_pair(first, second, active_plugs):
    """Identify robot contact with a harness part outside the intended branch set."""
    for payload, robot in ((first, second), (second, first)):
        parts, robot_parts = payload.split("::"), robot.split("::")
        if "wire_harness_multi" not in parts:
            continue
        index = parts.index("wire_harness_multi")
        if index + 1 >= len(parts):
            continue
        link = parts[index + 1]
        for arm in ("left_arm", "right_arm"):
            if arm not in robot_parts:
                continue
            if any(re.fullmatch(re.escape(p) + r"(?:_wire_\d+)?", link) for p in active_plugs):
                return None
            i = robot_parts.index(arm)
            return {
                "payload_link": link,
                "arm": arm,
                "arm_link": robot_parts[i + 1] if i + 1 < len(robot_parts) else None,
            }
    return None


def inactive_payload_nest_pair(first, second, active_plugs):
    """Only the current intended branch may contact the shared passive nest."""
    for payload, fixture in ((first, second), (second, first)):
        parts, other = payload.split("::"), fixture.split("::")
        if "wire_harness_multi" not in parts or "fixtures" not in other:
            continue
        index = parts.index("wire_harness_multi")
        if index + 1 >= len(parts) or not other[-1].startswith("regrasp_"):
            continue
        link = parts[index + 1]
        if not any(re.fullmatch(re.escape(p) + r"(?:_wire_\d+)?", link) for p in active_plugs):
            return {"payload_link": link, "fixture_collision": other[-1]}
    return None


class InactivePayloadContactGuard:
    """Reject an inactive harness part touching an arm or shared passive nest."""

    def __init__(self, active_plugs, report):
        self.active = frozenset(active_plugs)
        if not self.active or any(not re.fullmatch(r"P[1-9]\d*", p) for p in self.active):
            raise ValueError("Explicit nonempty intended plug identities required")
        if "inactive_payload_contact_guard" in report:
            raise ValueError("Do not reset a contact episode")
        self.report, self.failure = report, None
        self.summary = {
            "active_plugs": sorted(self.active),
            "checked_events": 0,
            "scope": "Fixed intended branch set for arm and passive-nest contacts; ordinary supply-carrier contact is not rejected. Counts may include sensor duplicates, never forces. Active-plug role checks remain separate.",
        }
        report["inactive_payload_contact_guard"] = self.summary

    def check(self):
        if self.failure is not None:
            raise RuntimeError("Unplanned non-active harness contact is latched")

    def observe(self, first, second, stamp, phase):
        self.check()
        if phase == "final_open":
            return
        self.summary["checked_events"] += 1
        violation = inactive_payload_arm_pair(
            first, second, self.active
        ) or inactive_payload_nest_pair(first, second, self.active)
        if violation is not None:
            self.failure = {
                "phase": phase,
                "sim_time_s": stamp if math.isfinite(stamp) else None,
                "collision1": first,
                "collision2": second,
                **violation,
                "active_plugs": sorted(self.active),
            }
            self.report["inactive_payload_contact_failure"] = copy.deepcopy(self.failure)
            self.check()


def accumulate_contact(records, phase, first, second, depths, sim_time):
    """Aggregate diagnostic contacts without treating penetration as force."""
    pair = tuple(sorted((first, second)))
    key = (phase, *pair)
    record = records.setdefault(
        key,
        {
            "phase": phase,
            "collision1": pair[0],
            "collision2": pair[1],
            "messages": 0,
            "first_sim_time_s": sim_time,
            "last_sim_time_s": sim_time,
            "max_depth_m": 0.0,
        },
    )
    record["messages"] += 1
    record["last_sim_time_s"] = sim_time
    record["max_depth_m"] = max(record["max_depth_m"], max(depths, default=0.0))


def add_arm_contact_sensors(world):
    plugin(world, ("contact", "Contact"))
    # Both arms are needed for jaw/object/fixture diagnostics. Inter-arm pairs
    # can therefore arrive from duplicate sensors; counts are not unique events.
    for name in ("left_arm", "right_arm"):
        model = world.find(f"model[@name='{name}']")
        if model is None:
            continue
        for link in model.findall("link"):
            for index, collision in enumerate(link.findall("collision")):
                sensor = child(link, "sensor", name=f"inter_arm_contact_{index}", type="contact")
                child(sensor, "always_on", "true")
                child(sensor, "update_rate", 30)
                child(sensor, "topic", CONTACT_TOPIC)
                contact = child(sensor, "contact")
                child(contact, "collision", collision.get("name"))
                child(contact, "topic", CONTACT_TOPIC)


def inter_arm_pair(first, second):
    """Names are scoped collision Entity.name fields, not numeric Gazebo IDs."""

    def belongs(name, arm):
        return any(part == arm or part.startswith(arm + "_") for part in name.split("::"))

    return (belongs(first, "left_arm") and belongs(second, "right_arm")) or (
        belongs(first, "right_arm") and belongs(second, "left_arm")
    )


def arm_fixture_pair(first, second):
    def arm(name):
        return any(
            part == a or part.startswith(a + "_")
            for part in name.split("::")
            for a in ("left_arm", "right_arm")
        )

    def fixture(name):
        return bool({"fixtures", "conveyor_carrier"} & set(name.split("::")))

    return arm(first) and fixture(second) or arm(second) and fixture(first)
