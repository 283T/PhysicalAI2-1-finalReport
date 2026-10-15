"""Native contact evaluation for a lifted connector; never generates motion targets."""

import copy
import json
import math


def connector_fixture_pair(first, second, plug):
    def connector(name):
        parts = name.split("::")
        return any(a == "wire_harness_multi" and b == plug for a, b in zip(parts, parts[1:]))

    def fixture(name):
        return bool({"fixtures", "conveyor_carrier"} & set(name.split("::")))

    if connector(first) and fixture(second):
        return second
    if connector(second) and fixture(first):
        return first
    return None


class CarriedContactGuard:
    """Latch unintended contacts, retaining their actual sensor timestamps.

    Register only after independent lift/hold succeeds and the object is clear
    of its source fixture. Permission for the exact drawing socket starts at
    a fresh descent timestamp. Late messages cannot gain retroactive permission.
    Duplicate sensors are diagnostic messages, not unique physical contacts.
    """

    def __init__(self, plug, socket, start_sim_time, report, stream):
        if plug not in ("P1", "P2", "P3") or socket not in ("J1", "J2", "J3"):
            raise ValueError("Known connector and validated drawing socket required")
        if not math.isfinite(start_sim_time):
            raise ValueError("Finite fresh lift timestamp required")
        self._plug, self._socket, self._start = plug, socket, start_sim_time
        self._allow_from, self._failed = None, False
        self._stream = stream
        self._target_messages = 0
        self._report = report
        self._summary = {"messages": 0, "target_messages": 0, "failed": False}
        if plug in report.setdefault("carried_contact_guards", {}):
            raise ValueError("Preserve the original carried contact episode")
        report["carried_contact_guards"][plug] = {
            "socket": socket,
            "start_sim_time_s": start_sim_time,
            "summary": self._summary,
        }

    def begin_descent(self, stamp):
        self.check()
        if self._allow_from is not None or not math.isfinite(stamp) or stamp < self._start:
            raise ValueError("Single fresh descent boundary required")
        self._allow_from = stamp
        self._report["carried_contact_guards"][self._plug]["allow_target_from_sim_time_s"] = stamp

    def observe(self, first, second, stamp, phase):
        fixture = connector_fixture_pair(first, second, self._plug)
        if fixture is None:
            return
        finite = math.isfinite(stamp)
        historical = finite and stamp < self._start
        target = any(p.startswith("socket_" + self._socket + "_") for p in fixture.split("::"))
        permitted = finite and self._allow_from is not None and stamp >= self._allow_from and target
        check = {"success": historical or permitted, "historical_before_lift": historical}
        record = {
            "plug": self._plug,
            "phase": phase,
            "sim_time_s": stamp if finite else None,
            "collision1": first,
            "collision2": second,
            "check": check,
        }
        self._summary["messages"] += 1
        if permitted:
            self._target_messages += 1
            self._summary["target_messages"] = self._target_messages
        if not check["success"]:
            self._failed = True
            self._summary["failed"] = True
            self._report.setdefault("carried_contact_failures", []).append(copy.deepcopy(record))
        # Latch failure before writing, so an I/O exception cannot erase it.
        self._stream.write(json.dumps(record, allow_nan=False) + "\n")

    def check(self):
        if self._failed:
            raise RuntimeError(self._plug + " contacted an unintended fixture")

    def require_target_contact(self):
        self.check()
        if not self._target_messages:
            raise RuntimeError("No native contact with the drawing-selected socket recorded")


def register_carried_contact_guard(session, plug, socket, report, stream):
    """Caller must validate connector sensors and successful physical lift first."""
    row = session.tick()
    if row is None:
        raise RuntimeError("Fresh robot state required for contact monitoring")
    guard = CarriedContactGuard(plug, socket, row["sim_time_s"], report, stream)
    session.contact_guards.append(guard)
    return guard
