import copy
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from perception_stream import StableConnectors, SyncedFrames, image_array


def message(nanosec):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=nanosec)))


def detections(yaw=0):
    return [
        {"plug_id": plug, "surface_xyz_m": [i * 0.1, 0, 0.12], "yaw_rad": yaw}
        for i, plug in enumerate(("P1", "P2", "P3"))
    ]


class PerceptionStreamTest(unittest.TestCase):
    def test_pairing_rejects_skew_and_replays_and_bounds_memory(self):
        frames = SyncedFrames(capacity=2)
        frames.add(message(1), "rgb")
        frames.add(message(2), "depth")
        self.assertIsNone(frames.take_latest())
        frames.add(message(2), "rgb")
        self.assertEqual(frames.take_latest()[0], 2)
        frames.add(message(1), "depth")
        frames.add(message(1), "rgb")
        self.assertIsNone(frames.take_latest())
        for stamp in range(3, 20):
            frames.add(message(stamp), "rgb")
        self.assertEqual(set(frames.images["rgb"]), {18, 19})

    def test_decodes_padded_rgb_and_big_endian_depth(self):
        rgb = SimpleNamespace(
            height=2,
            width=1,
            step=4,
            encoding="rgb8",
            is_bigendian=False,
            data=bytes([1, 2, 3, 99, 4, 5, 6, 99]),
        )
        self.assertEqual(image_array(rgb).tolist(), [[[3, 2, 1]], [[6, 5, 4]]])
        depth = SimpleNamespace(
            height=2,
            width=1,
            step=8,
            encoding="32FC1",
            is_bigendian=True,
            data=np.array([0.5, 99, 0.75, 99], dtype=">f4").tobytes(),
        )
        np.testing.assert_array_equal(image_array(depth), [[0.5], [0.75]])
        depth.encoding = "16UC1"
        with self.assertRaises(ValueError):
            image_array(depth)

    def test_stability_handles_wraparound_order_and_resets_on_missing_or_motion(self):
        stable = StableConnectors()
        self.assertFalse(stable.update(detections(np.pi - 0.01)))
        self.assertFalse(stable.update(list(reversed(detections(-np.pi + 0.01)))))
        self.assertTrue(stable.update(detections(np.pi)))
        self.assertFalse(stable.update(detections()[:2]))
        self.assertEqual(stable.count, 0)
        stable.update(detections())
        shifted = copy.deepcopy(detections())
        shifted[0]["surface_xyz_m"][0] += 0.01
        self.assertFalse(stable.update(shifted))
        self.assertEqual(stable.count, 1)
        self.assertFalse(stable.update(detections(0.2)))
        self.assertEqual(stable.count, 1)
        self.assertFalse(stable.update(detections() + detections()[:1]))
        self.assertEqual(stable.count, 0)
