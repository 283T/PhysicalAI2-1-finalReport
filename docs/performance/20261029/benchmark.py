"""Bounded initial-scene load ablation, not a robot-control or grasp-success benchmark."""
import argparse
import hashlib
import json
import os
import signal
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
CASES = [('gui_capture', True, True, True), ('gui_no_capture', True, True, False),
         ('headless_cameras', False, True, False), ('headless_no_images', False, False, False)]


def write(path, data):
    temp = path.with_name(path.name + '.tmp')
    temp.write_text(json.dumps(data, indent=2) + '\n'); temp.replace(path)


def owned(partition):
    rows = []
    for p in Path('/proc').glob('[0-9]*'):
        if int(p.name) == os.getpid(): continue
        try:
            env = (p/'environ').read_bytes().split(b'\0')
            if ('GZ_PARTITION='+partition).encode() not in env: continue
            cmd = (p/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            stat = (p/'stat').read_text().rsplit(')', 1)[1].split()
            rows.append({'pid': int(p.name), 'pgid': os.getpgid(int(p.name)), 'command': cmd,
                         'cpu_ticks': int(stat[11])+int(stat[12]), 'rss_pages': int(stat[21])})
        except (OSError, ValueError, IndexError): continue
    return rows


def stop(process):
    if process is None or process.poll() is not None: return
    os.killpg(process.pid, signal.SIGINT)
    try: process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)


def no_foreign_sim():
    rows = subprocess.check_output(['ps', '-eo', 'pid,args'], text=True).splitlines()
    conflicts = [r for r in rows if ('gz sim' in r or 'scripts/dual_pickup.py' in r
                                   or 'scripts/record_dual_hold.py' in r or 'parameter_bridge' in r)
                 and 'benchmark.py' not in r]
    if conflicts: raise RuntimeError('Existing simulation/control processes: '+str(conflicts))


def run_case(base, definition, args):
    from gz.msgs10.world_stats_pb2 import WorldStatistics
    from gz.transport13 import Node
    name, gui, images, capture = definition
    dest=base/name; dest.mkdir()
    tree=ET.parse(args.world); world=tree.getroot().find('world'); changes=[]
    for link in world.findall('.//link'):
        for sensor in list(link.findall('sensor')):
            if not images and sensor.get('type') in ('camera','depth_camera','rgbd_camera'):
                changes.append({'removed_sensor':sensor.get('name'),'type':sensor.get('type')});link.remove(sensor)
    if not images:
        for plugin in list(world.findall('plugin')):
            if plugin.get('name')=='gz::sim::systems::Sensors':
                changes.append({'removed_plugin':plugin.get('name')});world.remove(plugin)
    scene=dest/'world.sdf'
    if images: scene.write_bytes(args.world.read_bytes())
    else: tree.write(scene,encoding='utf-8',xml_declaration=True)
    partition=f'load-ablation-{os.getpid()}-{name}'; os.environ['GZ_PARTITION']=partition
    env=os.environ.copy(); env['ROS_DOMAIN_ID']='179'
    report={'case':name,'trial_state':'running','scope':'Uncontrolled initial-scene throughput only; not full task speed or grasp validation',
            'gui':gui,'image_sensors':images,'capture':capture,'partition':partition,
            'benchmark_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'gui_config_sha256':hashlib.sha256(args.gui_config.read_bytes()).hexdigest(),
            'source_world_sha256':hashlib.sha256(args.world.read_bytes()).hexdigest(),
            'world_sha256':hashlib.sha256(scene.read_bytes()).hexdigest(),'changes':changes,
            'max_step_size_s':float(world.findtext('physics/max_step_size')),
            'warmup_sim_s':args.warmup,'measurement_sim_s_requested':args.measure,'measurement_wall_limit_s':args.wall_limit}
    write(dest/'result.json',report)
    stats=[];lock=threading.Lock();node=Node();server=recorder=None;logs=[];subscription=False
    def receive(msg):
        row={'host_monotonic_s':time.monotonic(),'sim_time_s':msg.sim_time.sec+msg.sim_time.nsec*1e-9,
             'gazebo_real_time_s':msg.real_time.sec+msg.real_time.nsec*1e-9,
             'iterations':msg.iterations,'reported_rtf':msg.real_time_factor,'paused':msg.paused}
        with lock: stats.append(row)
    try:
        no_foreign_sim()
        subscription=node.subscribe(WorldStatistics,f"/world/{world.get('name')}/stats",receive)
        if not subscription:raise RuntimeError('Stats subscription failed')
        log=(dest/'server.log').open('w');logs.append(log)
        command=['gz','sim','-r',str(scene)]
        if gui: command+=['--gui-config',str(args.gui_config)]
        else: command.insert(2,'-s')
        launch=time.monotonic();server=subprocess.Popen(command,env=env,stdout=log,stderr=log,start_new_session=True)
        report['server_pid']=server.pid;write(dest/'result.json',report)
        if capture:
            log=(dest/'capture.log').open('w');logs.append(log)
            recorder=subprocess.Popen(['ffmpeg','-y','-loglevel','warning','-f','x11grab','-video_size','1280x900','-framerate','5','-i',':99','-an','-c:v','libx264','-preset','ultrafast','-crf','24','-pix_fmt','yuv420p',str(dest/'raw.mkv')],env=env,stdout=log,stderr=log,start_new_session=True)
        deadline=launch+args.wall_limit
        while True:
            if server.poll() is not None:raise RuntimeError('Server exited before measurement')
            if recorder and recorder.poll() is not None:raise RuntimeError('Capture exited before measurement')
            with lock: latest=stats[-1].copy() if stats else None
            if latest and latest['sim_time_s']>=args.warmup:break
            if time.monotonic()>deadline:raise TimeoutError('Warmup/statistics timeout')
            time.sleep(.05)
        first=latest; first_resources=owned(partition); start=time.monotonic()
        report['warmup_wall_s']=start-launch;report['start_stat']=first;report['start_resources']=first_resources;write(dest/'result.json',report)
        while True:
            if server.poll() is not None:raise RuntimeError('Server exited during measurement')
            if recorder and recorder.poll() is not None:raise RuntimeError('Capture exited during measurement')
            with lock: latest=stats[-1].copy()
            if latest['sim_time_s']-first['sim_time_s']>=args.measure:break
            if time.monotonic()-start>args.wall_limit:raise TimeoutError('Measurement wall limit exceeded')
            time.sleep(.05)
        final_resources=owned(partition); elapsed=latest['host_monotonic_s']-first['host_monotonic_s'];sim=latest['sim_time_s']-first['sim_time_s']
        if elapsed<=0 or sim<=0 or latest['iterations']<=first['iterations']:raise RuntimeError('Nonadvancing statistics')
        ticks=sum(r['cpu_ticks'] for r in final_resources)-sum(r['cpu_ticks'] for r in first_resources)
        report.update(trial_state='finished',measurement_completed=True,end_stat=latest,
            measured_wall_s=elapsed,measured_sim_s=sim,measured_rtf=sim/elapsed,
            iterations=latest['iterations']-first['iterations'],end_resources=final_resources,
            approximate_mean_cpu_cores=ticks/os.sysconf('SC_CLK_TCK')/elapsed,
            rss_snapshot_sum_bytes=sum(r['rss_pages'] for r in final_resources)*os.sysconf('SC_PAGE_SIZE'),
            resource_scope='Owned-process CPU tick deltas and summed RSS snapshots; shared pages may be counted repeatedly; not peak memory')
    except Exception as error:
        report.update(trial_state='finished',measurement_completed=False,error=f'{type(error).__name__}: {error}')
    finally:
        for p in [recorder,server]:
            try: stop(p)
            except Exception as error:report.setdefault('cleanup_errors',[]).append(str(error))
        remaining=owned(partition);receipt=[]
        for group in sorted(set(r['pgid'] for r in remaining)):
            if group==os.getpgrp():continue
            members=[r for r in remaining if r['pgid']==group]
            receipt.append({'group':group,'members':members,'action':'SIGINT after exact partition ownership check'})
            try:os.killpg(group,signal.SIGINT)
            except ProcessLookupError:pass
        deadline=time.monotonic()+8
        while owned(partition) and time.monotonic()<deadline:time.sleep(.1)
        report['owned_cleanup']=receipt;report['remaining_owned_processes']=owned(partition)
        if report['remaining_owned_processes']:report['measurement_completed']=False;report['cleanup_error']='Owned processes remain; stop batch'
        if subscription:node.unsubscribe(f"/world/{world.get('name')}/stats")
        for log in logs:log.close()
        with lock: saved=list(stats)
        (dest/'stats.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in saved));write(dest/'result.json',report)
    print(json.dumps({k:report.get(k) for k in ['case','measurement_completed','measured_wall_s','measured_sim_s','measured_rtf','error','cleanup_error']}),flush=True)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--world',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--gui-config',type=Path,default=ROOT/'sim/harness_recording_gui.config')
    parser.add_argument('--warmup',type=float,default=1);parser.add_argument('--measure',type=float,default=3);parser.add_argument('--wall-limit',type=float,default=180)
    parser.add_argument('--cases',nargs='+',choices=[r[0] for r in CASES],default=[r[0] for r in CASES]);args=parser.parse_args()
    if not all(v>0 for v in [args.warmup,args.measure,args.wall_limit]):parser.error('Positive intervals required')
    if args.output.exists():parser.error('Preserve earlier results; choose a new output')
    no_foreign_sim();args.output.mkdir(parents=True);results=[]
    for case in CASES:
        if case[0] not in args.cases:continue
        r=run_case(args.output,case,args);results.append(r)
        write(args.output/'summary.json',{'scope':'Exploratory initial-scene load ablation; one run per case, not assembly acceptance','results':results})
        if r.get('remaining_owned_processes'):raise RuntimeError('Cannot continue while owned processes remain')
    if not all(r.get('measurement_completed') for r in results):raise SystemExit(1)

if __name__=='__main__':main()
