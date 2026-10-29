import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from arm_kinematics import ArmKinematics
from arm_session import ArmSession


class CommandPoseCacheTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        path = Path(self.directory.name) / 'oblique.urdf'
        path.write_text('''<robot name="oblique"><joint name="q" type="revolute">
          <parent link="world"/><child link="tip"/><axis xyz="2 2 2"/>
          <limit lower="-3.14" upper="3.14"/></joint></robot>''')
        self.arm = ArmKinematics(path, 'tip')
        self.session = ArmSession.__new__(ArmSession)
        self.session.arms = {'arm': self.arm}
        self.session.targets = {'arm': np.array([0.3])}
        self.session._command_pose_cache = {}

    def test_oblique_axis_matches_known_120_degree_cycle_and_jacobian(self):
        q = np.array([2 * np.pi / 3])
        pose, jacobian = self.arm.forward(q, jacobian=True)
        np.testing.assert_allclose(pose[:3, :3], [[0, 0, 1], [1, 0, 0], [0, 1, 0]], atol=1e-14)
        np.testing.assert_allclose(pose[:3, 3], [0.08, 0, 0], atol=1e-14)
        other = self.arm.forward(q + 1e-6)
        np.testing.assert_allclose(jacobian[:3, 0], (other[:3, 3] - pose[:3, 3]) / 1e-6, atol=1e-7)

    def test_command_changes_in_place_refresh_geometry_and_returned_rows_are_independent(self):
        first = self.session.command_tcp_pose('arm')
        first[0][3] = 999
        np.testing.assert_allclose(self.session.command_tcp_pose('arm'), self.arm.forward([0.3]))
        self.session.targets['arm'][0] = 0.8
        np.testing.assert_allclose(self.session.command_tcp_pose('arm'), self.arm.forward([0.8]))

    def test_base_and_tool_changes_refresh_geometry(self):
        self.session.command_tcp_pose('arm')
        self.arm.base[0, 3] = 0.2
        np.testing.assert_allclose(self.session.command_tcp_pose('arm'), self.arm.forward([0.3]))
        self.arm.tool[2, 3] = 0.1
        np.testing.assert_allclose(self.session.command_tcp_pose('arm'), self.arm.forward([0.3]))

    def test_cached_command_never_bypasses_stale_feedback_rejection(self):
        self.session.command_tcp_pose('arm')
        self.session.poses = (0.0, 1.0, {})
        self.session.joints = {'arm': (0.0, {})}
        with patch('arm_session.time.monotonic', return_value=2):
            self.assertIsNone(self.session.snapshot())


if __name__ == '__main__':
    unittest.main()
