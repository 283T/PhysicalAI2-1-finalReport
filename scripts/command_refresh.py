"""Refresh persistent controller targets without flooding unchanged channels."""

import math


class CommandRefresh:
    def __init__(self, interval_s=0.1):
        if not math.isfinite(interval_s) or interval_s <= 0:
            raise ValueError("Positive finite refresh interval required")
        self.interval = interval_s
        self.sent = {}
        self.summary = {
            "refresh_interval_wall_s": interval_s,
            "published": 0,
            "unchanged_suppressed": 0,
        }

    def publish(self, channel, value, now, send):
        value = float(value)
        if not math.isfinite(value) or not math.isfinite(now):
            raise ValueError("Finite command and wall time required")
        previous = self.sent.get(channel)
        if previous is not None and value == previous[0] and 0 <= now - previous[1] < self.interval:
            self.summary["unchanged_suppressed"] += 1
            return False
        # Record only after a successful send; failed transport must be retried.
        send(value)
        self.sent[channel] = (value, now)
        self.summary["published"] += 1
        return True
