"""Recheck empty-gripper diagnostic convergence and every saved hold observation."""
import argparse
import gzip
import hashlib
import importlib.util
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT=Path(__file__).resolve().parents[4]


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--registry',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    if args.output.exists():parser.error('Preserve previous audits')
    registry=json.loads(args.registry.read_text());spec=importlib.util.spec_from_file_location('reference_fk',Path(__file__).parents[1]/'reference_arm_kinematics.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    source=json.loads((ROOT/registry['fixed_target_report']).read_text());goal=np.array(source['stop_diagnostics']['command_targets_rad']['right_arm']);audits=[]
    for case in registry['cases']:
        root=ROOT/case['output'];report=json.loads((root/'validation.json').read_text());assert report['trial_state']=='finished';assert not report['remaining_owned_processes'];assert report['world_sha256']==case['world_sha256'];assert report['method_sha256']==registry['method_sha256'];assert report['max_step_size_s']==case['step_s'];assert report['right_wrist_pid']['i_gain']==case['right_joint_6_i_gain'];np.testing.assert_array_equal(goal,report['goal_q_rad'])
        for p in report['preflights']:
            if 'execution_budget' in p:assert abs(p['execution_budget']['simulation_budget_s']-p['nominal_duration_s']-20)<1e-9
        audit={'case':case['label'],'reported_success':report['success'],'error':report.get('error'),'world_sha256':report['world_sha256'],'method_sha256':report['method_sha256'],'total_wall_s':report['total_wall_s'],'scope':'Isolated hover convergence only, not full pickup or formal acceptance'}
        if report['success']:
            world=ET.parse(ROOT/case['world']).getroot().find('world');base=[float(v) for v in world.findtext("model[@name='right_arm']/pose").split()];arm=module.ArmKinematics(ROOT/'outputs/stepped_hd_shm64_20261016/models/right_arm.urdf','right_arm_gripper',base);target=arm.forward(goal)
            windows=[w for w in report['dwell_windows'] if w['phase']=='diagnostic_hold'];assert len(windows)==1;w=windows[0];assert w['completed'] and w['end_sim_time_s']-w['start_sim_time_s']>=4
            maximum={'joint':0.,'position':0.,'angle':0.,'fk_native':0.};stamps=set();count=0
            with gzip.open(root/'hold_observations.jsonl.gz','rt') as stream:
                for line in stream:
                    row=json.loads(line);a=row['arms']['right_arm'];stamp=row['sim_time_s'];assert w['start_sim_time_s']-1e-8<=stamp<=w['end_sim_time_s']+1e-8;stamps.add(stamp);count+=1
                    q=np.array(a['q_rad']);native=np.array(a['tcp']);rotation=np.array(a['tcp_rotation_world']);angle=math.acos(float(np.clip((np.trace(target[:3,:3]@rotation.T)-1)/2,-1,1)))
                    for key,value in [('joint',np.max(np.abs(q-goal))),('position',np.linalg.norm(native-target[:3,3])),('angle',angle),('fk_native',np.linalg.norm(arm.forward(q)[:3,3]-native))]:maximum[key]=max(maximum[key],float(value))
            assert count==report['hold']['distinct_samples'] and len(stamps)>=10;assert maximum['joint']<.01 and maximum['position']<.002 and maximum['angle']<.025 and maximum['fk_native']<.001;assert not report['unintended_contacts']
            audit.update(independent_pass=True,observations=count,distinct_timestamps=len(stamps),hold_elapsed_sim_s=w['end_sim_time_s']-w['start_sim_time_s'],maximum_errors=maximum)
        else:
            audit['independent_pass']=False
            if report.get('stop_diagnostics'):
                d=report['stop_diagnostics'];a=d['last_observation']['arms']['right_arm'];error=float(np.max(np.abs(np.array(a['q_rad'])-goal)));audit['final_maximum_joint_error_rad']=error;audit['stop_reason']=d.get('execution_budget',{}).get('stop_reason')
            audit['failed_result_retained']=True
        audits.append(audit)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps({'scope':'Same-gate diagnostic audit; never added to formal B or declared a harness pickup success','registry_sha256':hashlib.sha256(args.registry.read_bytes()).hexdigest(),'cases':audits},indent=2)+'\n')
    print(json.dumps(audits))

if __name__=='__main__':main()
