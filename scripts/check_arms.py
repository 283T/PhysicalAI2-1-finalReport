"""Exercise both official robot models through ROS and verify independent joint feedback."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from rclpy.qos import qos_profile_sensor_data
from build_factory import joint_names

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--robot', choices=['ur3e','melfa_rv4frl'], default='ur3e')
    parser.add_argument('--world', type=Path, default=ROOT/'sim/worlds/arms_test.sdf')
    args=parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = '172'
    env = os.environ.copy()
    env['GZ_PARTITION'] = f'arms-check-{os.getpid()}'
    output = ROOT/'outputs'
    output.mkdir(exist_ok=True)
    processes = []
    rclpy.init()
    node = rclpy.create_node('dual_arm_check')
    states = {}
    publishers = {}
    bridge = ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge']
    for name in ('left_arm', 'right_arm'):
        def receive(msg, arm=name):
            states[arm] = (time.monotonic(), dict(zip(msg.name, msg.position)))
        node.create_subscription(JointState, f'/{name}/joints', receive, qos_profile_sensor_data)
        bridge.append(f'/{name}/joints@sensor_msgs/msg/JointState[gz.msgs.Model')
        for index in range(6):
            topic = f'/{name}/joint_{index}/cmd'
            publishers[name, index] = node.create_publisher(Float64, topic, 10)
            bridge.append(topic+'@std_msgs/msg/Float64]gz.msgs.Double')
    with (output/'arms_check.log').open('w') as log:
        try:
            for command in (['gz', 'sim', '-s', '-r', str(args.world)], bridge):
                processes.append(subprocess.Popen(command, env=env, stdout=log, stderr=log))
            deadline = time.monotonic()+25
            while len(states) < 2 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            if len(states) != 2:
                raise RuntimeError('Missing joint feedback from one or both robot models')
            initial = {name: data[1].copy() for name, data in states.items()}
            target = {'left_arm': .30, 'right_arm': -.25}
            start = time.monotonic()
            while time.monotonic()-start < 10:
                fraction = min(1., (time.monotonic()-start)/2)
                for (name, index), publisher in publishers.items():
                    publisher.publish(Float64(data=target[name]*fraction if index == 0 else 0.))
                rclpy.spin_once(node, timeout_sec=.05)
            results = {}
            for name, (stamp, positions) in states.items():
                expected = {name+'_'+joint: target[name] if i == 0 else 0.
                            for i, joint in enumerate(joint_names(args.robot))}
                errors = {joint: abs(positions.get(joint, float('inf'))-goal)
                          for joint, goal in expected.items()}
                results[name] = {'initial_rad': initial[name], 'final_rad': positions,
                                 'target_rad': expected, 'max_error_rad': max(errors.values()),
                                 'feedback_age_s': time.monotonic()-stamp,
                                 'success': max(errors.values()) < .025 and time.monotonic()-stamp < 1}
            result = {'robot': args.robot+' x 2', 'arms': results,
                      'success': all(arm['success'] for arm in results.values()),
                      'scope': 'Independent six-joint control under gravity; not grasping or insertion'}
            (output/('melfa_arms_validation.json' if args.robot == 'melfa_rv4frl' else 'arms_validation.json')).write_text(json.dumps(result, indent=2)+'\n')
            print(json.dumps(result, indent=2))
            if not result['success']:
                raise RuntimeError('Dual-arm position feedback validation failed')
        finally:
            for process in processes:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
