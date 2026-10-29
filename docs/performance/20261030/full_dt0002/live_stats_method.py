import json,os,time,threading,importlib.util
from pathlib import Path
p=Path('outputs/performance_pickup_dt0002_settle_20261030_r01');rec=json.loads((p/'recording.json').read_text());partition=rec['environment']['GZ_PARTITION'];os.environ['GZ_PARTITION']=partition
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node
spec=importlib.util.spec_from_file_location('bench','docs/performance/20261029/benchmark.py');bench=importlib.util.module_from_spec(spec);spec.loader.exec_module(bench)
rows=[];lock=threading.Lock()
def callback(msg):
 with lock:rows.append({'wall_monotonic_s':time.monotonic(),'sim_time_s':msg.sim_time.sec+msg.sim_time.nsec*1e-9,'iterations':msg.iterations,'reported_rtf':msg.real_time_factor})
node=Node();assert node.subscribe(WorldStatistics,'/world/multi_connector_factory/stats',callback)
start=time.monotonic();a=bench.owned(partition);phase=(p/'capture_caption.txt').read_text().strip();time.sleep(10);b=bench.owned(partition);elapsed=time.monotonic()-start
byid={row['pid']:row for row in a};cpu=[{'pid':row['pid'],'command':row['command'],'cpu_cores':(row['cpu_ticks']-byid[row['pid']]['cpu_ticks'])/os.sysconf('SC_CLK_TCK')/elapsed,'rss_bytes':row['rss_pages']*os.sysconf('SC_PAGE_SIZE')} for row in b if row['pid'] in byid]
with lock:stats=list(rows)
d={'scope':'Read-only live full-workcell 10s exploratory stats and owned-process CPU; not entire task or repeated benchmark','phase_at_start':phase,'phase_at_end':(p/'capture_caption.txt').read_text().strip(),'observation_wall_s':elapsed,'statistics':stats,'cpu':cpu}
if len(stats)>=2:d['observed_real_time_factor']=(stats[-1]['sim_time_s']-stats[0]['sim_time_s'])/(stats[-1]['wall_monotonic_s']-stats[0]['wall_monotonic_s'])
(p/'live_stats_r01.json').write_text(json.dumps(d,indent=2)+'\n');print(json.dumps({k:v for k,v in d.items() if k!='statistics'},indent=2))
