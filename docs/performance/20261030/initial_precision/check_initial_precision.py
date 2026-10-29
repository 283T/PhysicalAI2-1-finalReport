"""Observe both existing RGB-D views before any arm motion; diagnostic only."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'scripts'))
import rclpy
from arm_observation import acquire_observation
from arm_session import ArmSession
from evidence_io import write_json_atomic

spec = importlib.util.spec_from_file_location('bench', ROOT / 'docs/performance/20261029/benchmark.py')
bench = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bench)


def main():
    out = ROOT / 'outputs/initial_precision_20261030_r01'
    if out.exists():
        raise ValueError('Preserve previous diagnostic')
    bench.no_foreign_sim()
    out.mkdir()
    partition = 'initial-precision-' + str(os.getpid())
    os.environ.update(GZ_PARTITION=partition, ROS_DOMAIN_ID='175')
    world = ROOT / 'outputs/performance_pickup_dt0002_20261030_input/case/world.sdf'
    models = ROOT / 'outputs/stepped_hd_shm64_20261016/models'
    calibration = world.with_name('camera_calibration.json')
    report = {'scope': 'Fresh RGB-D diagnostic before arm motion; no grasp/hold or full-task success claim',
              'trial_state': 'running', 'partition': partition,
              'source_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in sorted((ROOT / 'scripts').glob('*.py'))},
              'method_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'world_sha256': hashlib.sha256(world.read_bytes()).hexdigest()}
    write_json_atomic(out / 'result.json', report)
    server = session = None
    rclpy.init()
    try:
        with (out / 'server.log').open('w') as scene_log, (out / 'bridge.log').open('w') as bridge_log:
            server = subprocess.Popen(['gz', 'sim', '-r', str(world), '--gui-config',
                                      'docs/performance/20261029/gui_800x600.config'],
                                     stdout=scene_log, stderr=scene_log, start_new_session=True)
            report['server_pid'] = server.pid
            write_json_atomic(out / 'result.json', report)
            session = ArmSession(world, models, bridge_log)
            session.observation_timeout_wall_s = 240
            session.wait_ready()
            session.dwell('initial_scene_settle', 2.5)
            session.phase = 'overview_observation'
            report['overview'] = acquire_observation(session, out / 'overview')
            session.phase = 'inspection_observation'
            report['inspection'] = acquire_observation(session, out / 'inspection', calibration=calibration)
            report['native_after_both_observations'] = session.snapshot()
            report['measurement_completed'] = True
    except Exception as error:
        report.update(measurement_completed=False, error=f'{type(error).__name__}: {error}')
    finally:
        if session:
            try:
                session.close()
            except Exception as error:
                report['session_cleanup_error'] = str(error)
        bench.stop(server)
        for group in {r['pgid'] for r in bench.owned(partition)}:
            if group != os.getpgrp():
                try:
                    os.killpg(group, 2)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + 8
        while bench.owned(partition) and time.monotonic() < deadline:
            time.sleep(.1)
        report['remaining_owned_processes'] = bench.owned(partition)
        report['trial_state'] = 'finished'
        report['changed_sources'] = [p for p, sha in report['source_sha256'].items()
                                     if hashlib.sha256((ROOT / p).read_bytes()).hexdigest() != sha]
        rclpy.shutdown()
        write_json_atomic(out / 'result.json', report)
    print(json.dumps({k: report.get(k) for k in ['measurement_completed', 'error', 'remaining_owned_processes', 'changed_sources']}))
    if not report.get('measurement_completed') or report['remaining_owned_processes'] or report['changed_sources']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
