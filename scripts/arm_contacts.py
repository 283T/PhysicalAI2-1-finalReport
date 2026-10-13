"""Contact sensors and inter-arm collision classification for the factory scene."""

from build_world import child, plugin

CONTACT_TOPIC = "/factory/arm_contacts"


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
