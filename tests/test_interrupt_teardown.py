"""A real ROS context must stay usable for final opening after Python SIGINT."""

import json
import os
import subprocess
import sys
import unittest


class InterruptTeardownTests(unittest.TestCase):
    def test_sigint_does_not_invalidate_final_ros_publish(self):
        code = """
import json,os,signal
import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64
rclpy.init(domain_id=99, signal_handler_options=SignalHandlerOptions.NO)
node=rclpy.create_node('test_interrupt_teardown')
publisher=node.create_publisher(Float64, '/test_interrupt_teardown_no_robot', 1)
interrupted=False
try:
    os.kill(os.getpid(), signal.SIGINT)
except KeyboardInterrupt:
    interrupted=True
finally:
    ok=rclpy.ok()
    publisher.publish(Float64(data=0.018))
    node.destroy_node()
    rclpy.shutdown()
print(json.dumps({'interrupted':interrupted, 'context_usable_for_final_publish':ok}))
"""
        env = {**os.environ, "ROS_LOCALHOST_ONLY": "1"}
        result = subprocess.run(
            [sys.executable, "-c", code],
            env=env,
            capture_output=True,
            text=True,
            timeout=20,
            check=True,
        )
        self.assertEqual(
            json.loads(result.stdout.strip().splitlines()[-1]),
            {"interrupted": True, "context_usable_for_final_publish": True},
        )
