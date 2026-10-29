"""Isolated empty-gripper camera-planned hover response; no pickup success claim."""
import argparse
import gzip
import hashlib
import importlib.util
import json
import os
import sys
import time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'scripts'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['world','models','output','target-report']:parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('Preserve previous results')
    spec=importlib.util.spec_from_file_location('bench',ROOT/'docs/performance/20261029/benchmark.py');bench=importlib.util.module_from_spec(spec);spec.loader.exec_module(bench);bench.no_foreign_sim()
    args.output.mkdir(parents=True);os.environ.update(GZ_PARTITION=f'wrist-convergence-{os.getpid()}',ROS_DOMAIN_ID='182',FASTRTPS_DEFAULT_PROFILES_FILE='/workspace/configs/fastdds_rgbd_hd64.xml')
    import numpy as np
    import rclpy
    import subprocess
    import xml.etree.ElementTree as ET
    from arm_session import ArmSession
    from cooperative_pick import upright_waist_path
    from arm_kinematics import rotation_error
    from evidence_io import write_json_atomic
    source=json.loads(args.target_report.read_text());goal=np.array(source['stop_diagnostics']['command_targets_rad']['right_arm'])
    world=ET.parse(args.world).getroot().find('world');plugin=next(p for p in world.find("model[@name='right_arm']").findall('plugin') if p.findtext('joint_name')=='right_arm_rv4frl_joint_6')
    report={'scope':'Empty-gripper right-arm hover convergence with harness and cameras removed; not a pickup, holding-success or full-workcell throughput trial','trial_state':'running','success':False,'world_sha256':hashlib.sha256(args.world.read_bytes()).hexdigest(),'source_target_report_sha256':hashlib.sha256(args.target_report.read_bytes()).hexdigest(),'target_origin':'Recorded command computed from camera recognition in prior development trial; no object ground-truth target','goal_q_rad':goal.tolist(),'max_step_size_s':float(world.findtext('physics/max_step_size')),'right_wrist_pid':{k:float(plugin.findtext(k)) for k in ['p_gain','i_gain','d_gain','cmd_max','cmd_min']},'joint_gate_rad_exclusive':.01,'settle_time_s':20,'requested_hold_sim_s':4,'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((ROOT/'scripts').glob('*.py'))},'method_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'environment':{k:os.environ[k] for k in ['GZ_PARTITION','ROS_DOMAIN_ID']}}
    write_json_atomic(args.output/'validation.json',report);server=session=None;started=time.monotonic();rclpy.init()
    try:
        with (args.output/'server.log').open('w') as server_log,(args.output/'bridge.log').open('w') as bridge_log:
            server=subprocess.Popen(['gz','sim','-s','-r',str(args.world)],stdout=server_log,stderr=server_log,start_new_session=True)
            report['server_pid']=server.pid;write_json_atomic(args.output/'validation.json',report)
            session=ArmSession(args.world,args.models,bridge_log);session.wait_ready();report['phase']='initial_open';write_json_atomic(args.output/'validation.json',report);session.dwell('initial_open',1)
            start=np.array(session.snapshot()['arms']['right_arm']['q_rad']);path=upright_waist_path(start,goal);report['command_path']=path.tolist();report['phase']='diagnostic_hover';write_json_atomic(args.output/'validation.json',report);move_wall=time.monotonic();session.move('diagnostic_hover',{'right_arm':path},settle_time_s=20);report['move_wall_s']=time.monotonic()-move_wall
            target=session.arms['right_arm'].forward(goal);report['phase']='diagnostic_hold';write_json_atomic(args.output/'validation.json',report);rows=session.dwell('diagnostic_hold',4);
            with gzip.open(args.output/'hold_observations.jsonl.gz','wt') as evidence:
                for row in rows:evidence.write(json.dumps(row)+'\n')
            max_joint=max_position=max_angle=max_fk=0.
            for row in rows:
                a=row['arms']['right_arm'];max_joint=max(max_joint,float(np.max(np.abs(np.array(a['q_rad'])-goal))));max_position=max(max_position,float(np.linalg.norm(np.array(a['tcp'])-target[:3,3])));max_angle=max(max_angle,float(np.linalg.norm(rotation_error(target[:3,:3],np.array(a['tcp_rotation_world'])))));max_fk=max(max_fk,a['fk_native_error_m'])
            report['hold']={'distinct_samples':len(rows),'max_joint_error_rad':max_joint,'max_tcp_position_error_m':max_position,'max_tcp_angle_error_rad':max_angle,'max_fk_native_error_m':max_fk,'gates':{'joint_exclusive':.01,'tcp_position_exclusive':.002,'tcp_angle_exclusive':.025,'fk_exclusive':.001}}
            report['success']=bool(len(rows)>=10 and max_joint<.01 and max_position<.002 and max_angle<.025 and max_fk<.001)
            if not report['success']:report['error']='Post-convergence hold outside unchanged gates'
    except Exception as error:
        report.update(success=False,error=f'{type(error).__name__}: {error}')
    finally:
        report['control_wall_s_before_teardown']=time.monotonic()-started
        if session:
            try:session.close()
            except Exception as error:report.update(success=False,teardown_error=str(error))
            report.update(stop_diagnostics=session.stop_diagnostics,preflights=session.preflights,dwell_windows=session.dwell_windows,unintended_contacts=session.unintended,contacts=list(session.contacts.values()))
            (args.output/'trajectory.json.gz').write_bytes(gzip.compress(json.dumps(session.history).encode(),mtime=0))
        try:bench.stop(server)
        except Exception as error:report.update(success=False,cleanup_error=str(error))
        partition=os.environ['GZ_PARTITION'];left=bench.owned(partition)
        for group in {r['pgid'] for r in left}:
            if group!=os.getpgrp():
                try:os.killpg(group,2)
                except ProcessLookupError:pass
        deadline=time.monotonic()+8
        while bench.owned(partition) and time.monotonic()<deadline:time.sleep(.1)
        report['remaining_owned_processes']=bench.owned(partition)
        if report['remaining_owned_processes']:report['success']=False
        report.update(trial_state='finished',total_wall_s=time.monotonic()-started);rclpy.shutdown();write_json_atomic(args.output/'validation.json',report)
    print(json.dumps({k:report.get(k) for k in ['trial_state','success','error','move_wall_s','total_wall_s','hold']}))
    if not report['success']:raise SystemExit(1)

if __name__=='__main__':main()
