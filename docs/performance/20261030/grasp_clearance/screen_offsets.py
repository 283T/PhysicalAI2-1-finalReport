"""Offline clearance candidates from failed measured arm state; no simulation commands."""
import gzip
import json
from pathlib import Path
import sys
import numpy as np
ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'scripts'))
from plan_reach import load_arms
from arm_collision_bounds import ArmCollisionBounds, FingerStateBounds, screen_paths
from fixture_collision_bounds import FixtureBounds
from cooperative_pick import shared_lift
from gripper_model import gap_m

trial = ROOT / 'outputs/performance_pickup_dt0005_i20_j2i200_precision_20261030_r02/pickup'
world = ROOT / 'outputs/performance_pickup_dt0005_i20_j2i200_20261030_input/case/world.sdf'
models = ROOT / 'outputs/stepped_hd_shm64_20261016/models'
report = json.loads((trial / 'validation.json').read_text())
trace = json.loads(gzip.decompress((trial / 'trajectory.json.gz').read_bytes()))
state = next(row for row in trace if row['phase'] == 'primary_lift')
arms = load_arms(world, models)
q = {n: np.asarray(a['q_rad']) for n, a in state['arms'].items()}
intervals = report['preflights'][-1]['planned_finger_intervals_m']
bounds = {n: FingerStateBounds(ArmCollisionBounds(models / (n + '.urdf'), arm),
          {n + '_finger_' + side + '_joint': intervals[n][i]
           for i, side in enumerate(['left', 'right'])}) for n, arm in arms.items()}
fixtures = FixtureBounds(world, include_carrier=True)
start = arms['right_arm'].forward(q['right_arm'])
rows = []
for distance in [0., .001, .002, .003, .004]:
    target = start.copy()
    target[:3, 3] -= target[:3, 2] * distance
    result = arms['right_arm'].inverse(target, seed=q['right_arm'], attempts=1,
                                       position_tolerance=.0001, angle_tolerance=.002)
    row = {'offset_along_outward_tool_axis_m': distance, 'ik_success': result['success']}
    if result['success']:
        joints = np.asarray(result['q_rad']) if distance else q['right_arm']
        _, lift = shared_lift({'right_arm': arms['right_arm']}, {'right_arm': joints})
        paths = {**lift, 'left_arm': np.array([q['left_arm'], q['left_arm']])}
        row.update(lift_fixture_screen=fixtures.screen(bounds, paths, margin=.001),
                   lift_arm_screen=screen_paths(bounds, paths),
                   hypothetical_right_q_rad=joints.tolist())
    rows.append(row)
print(json.dumps({'scope': 'Offline geometry only, based on failed measured joints and closed finger envelope. No object poses used. Does not validate actual grasp, trajectory from startup, or physics. Never restore these joints in a simulation.',
                  'source_sim_time_s': state['sim_time_s'], 'candidates': rows}, indent=2))
