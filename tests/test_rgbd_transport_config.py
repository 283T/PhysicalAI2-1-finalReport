"""Prevent a camera-resolution change from silently exceeding DDS image buffers."""

import json
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class RGBDTransportCapacityTest(unittest.TestCase):
    def test_dds_buffers_fit_full_size_sensor_messages_and_five_pairs(self):
        root = ET.parse(ROOT / "configs/fastdds_rgbd.xml").getroot()
        ns = {"d": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
        shm = next(
            row
            for row in root.findall(".//d:transport_descriptor", ns)
            if row.findtext("d:type", namespaces=ns) == "SHM"
        )
        message_size = int(shm.findtext("d:maxMessageSize", namespaces=ns))
        segment_size = int(shm.findtext("d:segment_size", namespaces=ns))
        for name in ("infeed_camera.json", "inspection_camera.json", "end_inspection_camera.json"):
            camera = json.loads((ROOT / "configs" / name).read_text())
            pixels = camera["width"] * camera["height"]
            with self.subTest(camera=name):
                self.assertGreaterEqual(message_size, pixels * 4 + 65536)
                self.assertGreaterEqual(segment_size, pixels * (3 + 4) * 5)
