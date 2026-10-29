"""Independently check completed scene statistics and summarize exploratory load ablation."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('source',type=Path);parser.add_argument('destination',type=Path);args=parser.parse_args()
    if args.destination.exists():parser.error('Preserve previous result archives')
    reports=[]
    for p in args.source.glob('*/result.json'):
        r=json.loads(p.read_text())
        if r['trial_state']!='finished':raise ValueError('Still running: '+r['case'])
        if r.get('measurement_completed'):
            rows=[json.loads(s) for s in (p.parent/'stats.jsonl').read_text().splitlines()]
            first,last=r['start_stat'],r['end_stat'];window=[s for s in rows if first['host_monotonic_s']<=s['host_monotonic_s']<=last['host_monotonic_s']]
            assert len(window)>=2 and first in window and last in window
            assert not any(s['paused'] for s in window)
            assert all(b['host_monotonic_s']>a['host_monotonic_s'] and b['iterations']>=a['iterations'] for a,b in zip(window,window[1:]))
            dt=last['sim_time_s']-first['sim_time_s'];wall=last['host_monotonic_s']-first['host_monotonic_s']
            assert dt>=r['measurement_sim_s_requested'] and abs(dt-r['measured_sim_s'])<1e-8
            assert abs(wall-r['measured_wall_s'])<1e-8 and abs(dt/wall-r['measured_rtf'])<1e-10
            assert abs(r['iterations']*r['max_step_size_s']-dt)<1e-7
            assert not r['remaining_owned_processes']
            r['independent_statistics_check']={'passed':True,'window_samples':len(window),'iteration_step_agrees_with_sim_delta':True,'unpaused':True}
        reports.append(r)
    reports.sort(key=lambda r:['gui_capture','gui_no_capture','headless_cameras','headless_no_images'].index(r['case']))
    baseline=next(r for r in reports if r['case']=='gui_capture')
    args.destination.mkdir(parents=True);manifest={}
    for p in sorted(args.source.rglob('*')):
        if not p.is_file() or p.suffix=='.mkv':continue
        rel=p.relative_to(args.source);data=p.read_bytes();target=args.destination/rel;encoding='unchanged'
        if p.suffix=='.sdf':target=target.with_suffix('.sdf.gz');encoding='gzip';stored=gzip.compress(data,mtime=0)
        else:stored=data
        target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(stored)
        assert (gzip.decompress(stored) if encoding=='gzip' else stored)==data
        manifest[str(rel)]={'sha256_original':hashlib.sha256(data).hexdigest(),'archive_path':str(target.relative_to(args.destination)),'encoding':encoding,'sha256_archive':hashlib.sha256(stored).hexdigest()}
    (args.destination/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    summary={'scope':'Exploratory uncontrolled initial scene, one run per case; no conclusion about contact-held/full task speed or correctness','trials':reports,'reference':'gui_capture','order':[r['case'] for r in reports]}
    (args.destination/'checked_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    lines=['| 構成 | 物理秒 | 実時間秒 | 速度比 RTF | 基準比 | CPU参考（コア） |','| --- | ---: | ---: | ---: | ---: | ---: |']
    for r in reports:
        if r.get('measurement_completed'):
            lines.append(f"| {r['case']} | {r['measured_sim_s']:.4f} | {r['measured_wall_s']:.2f} | {r['measured_rtf']:.4f} | {r['measured_rtf']/baseline['measured_rtf']:.2f}× | {r['approximate_mean_cpu_cores']:.2f} |")
        else:lines.append(f"| {r['case']} | 失敗 | — | — | — | — |")
    print('\n'.join(lines))

if __name__=='__main__':main()
