"""Read-only post-trial TCP drift check on the saved downsampled trajectory."""
import gzip
import json
from pathlib import Path
import numpy as np

trial = Path('outputs/performance_pickup_dt0002_inspection_20261030_r02/pickup')
rows = json.loads(gzip.decompress((trial / 'trajectory.json.gz').read_bytes()))
result = {'scope': 'Post-trial downsampled trace only; not a causal proof or controller input', 'phases': []}
for phase in ['primary_preclose_0', 'primary_preclose_1', 'primary_preclose_2']:
    samples = [row for row in rows if row['phase'] == phase]
    if not samples:
        continue
    first, last = samples[0], samples[-1]
    a, b = first['arms']['right_arm'], last['arms']['right_arm']
    result['phases'].append({'phase': phase, 'samples': len(samples),
        'first_sim_time_s': first['sim_time_s'], 'last_sim_time_s': last['sim_time_s'],
        'tcp_displacement_m': float(np.linalg.norm(np.asarray(a['tcp']) - b['tcp'])),
        'maximum_command_joint_change_rad': float(max(np.max(np.abs(
            np.asarray(row['arms']['right_arm']['command_q_rad']) - a['command_q_rad']))
            for row in samples)),
        'initial_tracking_error_m': float(np.linalg.norm(np.asarray(a['tcp']) - np.asarray(a['command_tcp_pose'])[:3, 3])),
        'final_tracking_error_m': float(np.linalg.norm(np.asarray(b['tcp']) - np.asarray(b['command_tcp_pose'])[:3, 3]))})
print(json.dumps(result, indent=2))
