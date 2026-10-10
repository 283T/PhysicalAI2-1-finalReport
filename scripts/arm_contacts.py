"""Contact sensors and inter-arm collision classification for the factory scene."""

from build_world import child, plugin

CONTACT_TOPIC = "/factory/arm_contacts"


def add_arm_contact_sensors(world):
    plugin(world, ("contact", "Contact"))
    # Monitoring every left-arm collision is sufficient to observe both sides
    # of any left/right-arm contact without duplicating all right-arm sensors.
    model = world.find("model[@name='left_arm']")
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
