"""Bounded idle-controller profiling, never counted as a pickup/holding trial."""
import argparse
import cProfile
import hashlib
import importlib.util
import json
import os
import pstats
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'scripts'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world',type=Path,required=True);parser.add_argument('--models',type=Path,required=True)
    parser.add_argument('--gui-config',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=10);args=parser.parse_args()
    if not 1<=args.seconds<=30:parser.error('Profile windows must be 1 to 30 seconds')
    if args.output.exists():parser.error('Use a new output directory')
    spec=importlib.util.spec_from_file_location('scene_benchmark',ROOT/'docs/performance/20261029/benchmark.py')
    bench=importlib.util.module_from_spec(spec);spec.loader.exec_module(bench);bench.no_foreign_sim()
    args.output.mkdir(parents=True);partition=f'controller-profile-{os.getpid()}'
    os.environ.update(GZ_PARTITION=partition,ROS_DOMAIN_ID='181',FASTRTPS_DEFAULT_PROFILES_FILE='/workspace/configs/fastdds_rgbd_hd64.xml')
    import rclpy
    from arm_session import ArmSession
    from evidence_io import write_json_atomic

    class ProfiledSession(ArmSession):
        def __init__(self,*a,**kw):
            self.profile_active=False;self.main_thread=threading.get_ident();self.callback_profiles={};self.callback_lock=threading.Lock()
            super().__init__(*a,**kw)
        def call_profiled(self,method,*a):
            if not self.profile_active or threading.get_ident()==self.main_thread:return method(*a)
            start=time.perf_counter()
            try:return method(*a)
            finally:
                elapsed=time.perf_counter()-start
                with self.callback_lock:
                    row=self.callback_profiles.setdefault(method.__name__,{'calls':0,'elapsed_s':0.0})
                    row['calls']+=1;row['elapsed_s']+=elapsed
        def receive_poses(self,msg):return self.call_profiled(super().receive_poses,msg)
        def receive_contacts(self,msg):return self.call_profiled(super().receive_contacts,msg)
        def receive_joints(self,name,msg):return self.call_profiled(super().receive_joints,name,msg)

    report={'scope':'Idle initial-target control tick and callback profile only; no grasp/movement or assembly acceptance',
            'trial_state':'running','seconds_per_window':args.seconds,'partition':partition,
            'physics_step_s':.0001,'source_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((ROOT/'scripts').glob('*.py'))},
            'world_sha256':hashlib.sha256(args.world.read_bytes()).hexdigest(),
            'gui_config_sha256':hashlib.sha256(args.gui_config.read_bytes()).hexdigest(),
            'method_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    write_json_atomic(args.output/'result.json',report);server=session=None;profile=cProfile.Profile();rclpy.init()
    try:
        with (args.output/'server.log').open('w') as server_log,(args.output/'bridge.log').open('w') as bridge_log:
            server=subprocess.Popen(['gz','sim','-r',str(args.world),'--gui-config',str(args.gui_config)],stdout=server_log,stderr=server_log,start_new_session=True)
            report['server_pid']=server.pid;write_json_atomic(args.output/'result.json',report)
            session=ProfiledSession(args.world,args.models,bridge_log);session.wait_ready()
            def window(label):
                row=session.snapshot();start_sim=row['sim_time_s'];start=time.monotonic();ticks=0
                while time.monotonic()-start<args.seconds:
                    session.tick();ticks+=1
                row=session.snapshot()
                if row is None:raise RuntimeError('No fresh final observation')
                return {'label':label,'ticks':ticks,'wall_s':time.monotonic()-start,'sim_s':row['sim_time_s']-start_sim,'joint_targets':'initial observed positions, fingers open'}
            report['unprofiled_window']=window('unprofiled')
            session.profile_active=True;profile.enable()
            try:report['profiled_window']=window('profiled')
            finally:profile.disable();session.profile_active=False
            profile.dump_stats(str(args.output/'main.pstats'))
            stats=pstats.Stats(str(args.output/'main.pstats'))
            with session.callback_lock:report['native_callback_timings']=dict(session.callback_profiles)
            rows=[]
            for (file,line,name),(primitive,calls,self_s,cumulative_s,callers) in stats.stats.items():
                rows.append({'file':file.replace(str(ROOT),'<project>'),'line':line,'function':name,'primitive_calls':primitive,'calls':calls,'self_s':self_s,'cumulative_s':cumulative_s})
            report.update(trial_state='finished',measurement_completed=True,
                top_self=sorted(rows,key=lambda r:r['self_s'],reverse=True)[:30],
                top_cumulative=sorted(rows,key=lambda r:r['cumulative_s'],reverse=True)[:30],
                profile_scope='cProfile covers main controller thread. Native callback elapsed time is measured separately without cProfile; instrumentation changes execution speed and times may overlap across threads.')
            with (args.output/'profile.txt').open('w') as f:
                stats.stream=f;stats.sort_stats('cumulative').print_stats(35);stats.sort_stats('tottime').print_stats(35)
    except Exception as error:
        report.update(trial_state='finished',measurement_completed=False,error=f'{type(error).__name__}: {error}')
    finally:
        if session:
            session.profile_active=False
            try:session.close()
            except Exception as error:report['session_cleanup_error']=str(error)
        try:bench.stop(server)
        except Exception as error:report['server_cleanup_error']=str(error)
        remaining=bench.owned(partition)
        for group in set(r['pgid'] for r in remaining):
            if group!=os.getpgrp():
                try:os.killpg(group,2)
                except ProcessLookupError:pass
        deadline=time.monotonic()+8
        while bench.owned(partition) and time.monotonic()<deadline:time.sleep(.1)
        report['remaining_owned_processes']=bench.owned(partition)
        if report['remaining_owned_processes']:report['measurement_completed']=False
        rclpy.shutdown();write_json_atomic(args.output/'result.json',report)
    print(json.dumps({k:report.get(k) for k in ['measurement_completed','unprofiled_window','profiled_window','error','remaining_owned_processes']}))
    if not report.get('measurement_completed'):raise SystemExit(1)

if __name__=='__main__':main()
