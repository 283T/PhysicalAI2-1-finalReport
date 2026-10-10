"""Verify dynamic gravity sag of the jointed cable independently of robot control."""
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
    os.environ['ROS_DOMAIN_ID']='171'
    env=os.environ.copy(); env['GZ_PARTITION']=f'cable-check-{os.getpid()}'
    output=ROOT/'outputs'; output.mkdir(exist_ok=True)
    processes=[]
    rclpy.init(); node=rclpy.create_node('cable_check'); samples=[]
    node.create_subscription(PoseArray,'/cable/poses',lambda msg:samples.append(
        {'time':time.monotonic(),'z':[p.position.z for p in msg.poses]}),qos_profile_sensor_data)
    with (output/'cable_check.log').open('w') as log:
        try:
            for cmd in ([ 'gz','sim','-s','-r',str(ROOT/'sim/worlds/cable_test.sdf')],
                        ['ros2','run','ros_gz_bridge','parameter_bridge','/cable/poses@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V']):
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
            (output/'cable_validation.json').write_text(json.dumps(result,indent=2)+'\n'); print(json.dumps(result,indent=2))
            if not result['success']:raise RuntimeError('Cable deformation test failed')
        finally:
            for process in processes:
                process.terminate()
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:process.kill();process.wait()
            node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
