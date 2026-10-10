"""Independent evaluator: ground-truth poses are never provided to the detector."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import signal

import numpy as np
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node


def rotation(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for name in ('validation.json', 'evaluation_truth.json'):
        (args.output/name).unlink(missing_ok=True)
    transport = Node()
    samples = []
    def receive(msg):
        entries = {p.name: {'xyz': [p.position.x, p.position.y, p.position.z],
                           'q': [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]}
                   for p in msg.pose if p.name in ('wire_harness_multi', 'P1', 'P2', 'P3')}
        if len(entries) == 4:
            samples.append({'stamp_ns': msg.header.stamp.sec*10**9+msg.header.stamp.nsec, 'poses': entries})
            if len(samples) > 2000:
                del samples[:1000]
    transport.subscribe(Pose_V, '/world/multi_connector_factory/dynamic_pose/info', receive)
    process = subprocess.Popen(['python3', 'scripts/observe_connectors.py', '--bridge', '--output', str(args.output)], start_new_session=True)
    try:
        process.wait(timeout=105)
        if process.returncode:
            raise RuntimeError('Perception run failed')
        detections = json.loads((args.output/'detections.json').read_text())
        if not samples:
            raise RuntimeError('No independent ground truth received')
        snapshot = min(samples, key=lambda sample: abs(sample['stamp_ns']-detections['sensor_stamp_ns']))
        skew = abs(snapshot['stamp_ns']-detections['sensor_stamp_ns'])/1e9
        if skew > .1:
            raise RuntimeError('Ground truth is not synchronized to the sensor image')
        poses = snapshot['poses']
        model = poses['wire_harness_multi']
        model_rotation = rotation(model['q'])
        rows = []
        for detection in detections['connectors']:
            plug = poses[detection['plug_id']]
            center = np.asarray(model['xyz'])+model_rotation@np.asarray(plug['xyz'])
            orientation = model_rotation@rotation(plug['q'])
            direction = orientation[:, 0]
            yaw = float(np.arctan2(direction[1], direction[0]))
            # Compare XY to the housing center; perceived Z is the visible top face, not center.
            xy_error = float(np.linalg.norm(center[:2]-np.asarray(detection['surface_xyz_m'])[:2]))
            angle_error = abs(float(np.arctan2(np.sin(detection['yaw_rad']-yaw), np.cos(detection['yaw_rad']-yaw))))
            rows.append({'plug_id': detection['plug_id'], 'true_center_xyz_m': center.tolist(),
                         'xy_error_m': xy_error, 'yaw_error_rad': angle_error,
                         'surface_above_center_m': detection['surface_xyz_m'][2]-center[2]})
        result = {'success': len(rows) == 3 and all(row['xy_error_m'] < .008 and row['yaw_error_rad'] < .15 for row in rows),
                  'stamp_skew_s': skew, 'connectors': rows,
                  'scope': 'RGB-D XY and directed planar yaw; ideal extrinsics, colored marked prototypes; no 6D pose or robot grasp'}
        (args.output/'validation.json').write_text(json.dumps(result, indent=2)+'\n')
        (args.output/'evaluation_truth.json').write_text(json.dumps(snapshot, indent=2)+'\n')
        print(json.dumps(result, indent=2))
        if not result['success']:
            raise RuntimeError('Perception accuracy validation failed')
    finally:
        # Release the native subscriber before Python finalizes the callback runtime.
        transport.unsubscribe('/world/multi_connector_factory/dynamic_pose/info')
        del transport
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


if __name__ == '__main__':
    main()
