"""Verify dynamic gravity sag of the jointed cable independently of robot control."""
import argparse
import math
import json
import os
from pathlib import Path
import subprocess
import time
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.qos import qos_profile_sensor_data

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--case-world', type=Path)
    args=parser.parse_args()
    os.environ['ROS_DOMAIN_ID']='171'
    env=os.environ.copy(); env['GZ_PARTITION']=f'cable-check-{os.getpid()}'
    output=ROOT/'outputs'; output.mkdir(exist_ok=True)
    processes=[]
    rclpy.init(); node=rclpy.create_node('cable_check'); samples=[]
    node.create_subscription(PoseArray,'/multi_harness/poses' if args.case_world else '/cable/poses',lambda msg:samples.append(
        {'time':time.monotonic(),'z':[p.position.z for p in msg.poses]}),qos_profile_sensor_data)
    world=ROOT/'sim/worlds/cable_test.sdf'
    topic='/cable/poses'
    if args.case_world:
        import xml.etree.ElementTree as ET
        tree=ET.parse(args.case_world)
        scene=tree.getroot().find('world')
        # Isolate contact/deformation from arm control and GPU rendering.
        for model in list(scene.findall('model')):
            if model.get('name') in ('left_arm', 'right_arm', 'overhead_camera'):
                scene.remove(model)
        for system in list(scene.findall('plugin')):
            if system.get('name') == 'gz::sim::systems::Sensors': scene.remove(system)
        world=output/'multi_harness_physics.sdf'
        tree.write(world, encoding='utf-8', xml_declaration=True)
        topic='/multi_harness/poses'
    with (output/'cable_check.log').open('w') as log:
        try:
            for cmd in ([ 'gz','sim','-s','-r',str(world)],
                        ['ros2','run','ros_gz_bridge','parameter_bridge',topic+'@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V']):
                processes.append(subprocess.Popen(cmd,env=env,stdout=log,stderr=log))
            end=time.monotonic()+15
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
            last=samples[-1]['z'] if samples else []
            # PosePublisher reports these link poses relative to the model frame.
            # The anchored body is at local z=0. PoseArray drops names and does not preserve link order.
            anchor_present=any(abs(z)<1e-8 for z in last)
            sag=-min(last) if last else 0
            result={'link_count':len(last),'gravity_sag_m':sag,'samples':len(samples),'link_z_m':last,
                    'reference':'model-local z=0 anchored body; link order is not assumed',
                    'success':len(last)==13 and anchor_present and .03<sag<.35,
                    'scope':'jointed rigid segments with bending springs; not a calibrated continuum material'}
            if args.case_world:
                span=max(last)-min(last) if last else 0
                result={'link_count':len(last),'vertical_span_m':span,'samples':len(samples),
                        'success':len(last)==31 and all(math.isfinite(z) for z in last) and .005<span<.5,
                        'scope':'Three-plug gravity/contact deformation smoke test; no stability/material validation or robot connection',
                        'case_world':str(args.case_world)}
            (output/('multi_harness_validation.json' if args.case_world else 'cable_validation.json')).write_text(json.dumps(result,indent=2)+'\n'); print(json.dumps(result,indent=2))
            if not result['success']:raise RuntimeError('Cable deformation test failed')
        finally:
            for process in processes:
                process.terminate()
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:process.kill();process.wait()
            node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
