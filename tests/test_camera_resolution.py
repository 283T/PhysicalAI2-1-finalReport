import json
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from resize_inspection_camera import resize


class CameraResolutionTests(unittest.TestCase):
    def test_resize_preserves_scene_and_rejects_source_disagreement(self):
        with tempfile.TemporaryDirectory() as directory:
            case = Path(directory) / "source"
            case.mkdir()
            original = '<sdf><world><physics><max_step_size>0.0001</max_step_size></physics><model name="arm"><pose>0 0 0 0 0 0</pose></model><model name="camera"><link><sensor type="rgbd_camera"><topic>/test/rgbd</topic><camera><image><width>960</width><height>720</height></image></camera></sensor></link></model></world></sdf>'
            (case / "world.sdf").write_text(original)
            calibration = Path(directory) / "calibration.json"
            config = {
                "frame": "world",
                "position_m": [0, 0, 0],
                "pose_rpy_rad": [0, 0, 0],
                "optical_to_world_rotation": [[0, 0, 1], [-1, 0, 0], [0, -1, 0]],
                "horizontal_fov_rad": 1.1,
                "width": 960,
                "height": 720,
                "topic": "/test/rgbd",
            }
            calibration.write_text(json.dumps(config))
            output = Path(directory) / "resized"
            resize(case, calibration, output, 2)
            tree = ET.parse(output / "world.sdf")
            image = tree.find("world/model/link/sensor/camera/image")
            self.assertEqual(image.findtext("width"), "1920")
            self.assertEqual(image.findtext("height"), "1440")
            image.find("width").text = "960"
            image.find("height").text = "720"
            for node in tree.iter():
                node.tail = None
                if node.text is not None and not node.text.strip():
                    node.text = None
            self.assertEqual(ET.tostring(tree.getroot()), ET.tostring(ET.fromstring(original)))
            self.assertEqual((case / "world.sdf").read_text(), original)
            self.assertEqual(
                json.loads((output / "camera_calibration.json").read_text())["width"], 1920
            )
            config["width"] = 640
            calibration.write_text(json.dumps(config))
            with self.assertRaises(ValueError):
                resize(case, calibration, Path(directory) / "mismatch", 2)
            with self.assertRaises(ValueError):
                resize(case, calibration, output, 2)
            with self.assertRaises(ValueError):
                resize(case, calibration, Path(directory) / "invalid", 0)

    def test_large_dds_profile_carries_full_resolution_depth_payload(self):
        path = Path(__file__).resolve().parents[1] / "configs/fastdds_rgbd_large.xml"
        root = ET.parse(path)
        values = {e.tag.rsplit("}", 1)[-1]: e.text for e in root.iter()}
        self.assertGreater(int(values["maxMessageSize"]), 1920 * 1440 * 4 + 1024)
        self.assertGreaterEqual(int(values["segment_size"]), 3 * 1920 * 1440 * (3 + 4))
