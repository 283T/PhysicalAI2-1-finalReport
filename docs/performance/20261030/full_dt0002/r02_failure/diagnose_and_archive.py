import sys,json,gzip,hashlib,shutil,subprocess
from pathlib import Path
import numpy as np
sys.path.insert(0,'scripts')
from arm_kinematics import quaternion_rotation,rotation_error
r=Path('outputs/performance_pickup_dt0002_inspection_20261030_r02');p=r/'pickup'
t=json.loads(gzip.decompress((p/'trajectory.json.gz').read_bytes()))
rows=[]
for phase in ['primary_observation','primary_initial_inspection','primary_preclose_0','primary_preclose_1','primary_preclose_2']:
 d=json.loads((p/phase/'detections.json').read_text());stamp=d['sensor_stamp_ns']/1e9
 row=min(t,key=lambda x:abs(x['sim_time_s']-stamp));poses=row['evaluation_poses'];m=poses['wire_harness_multi'];link=poses['P3'];mr=quaternion_rotation(m['q']);nr=mr@quaternion_rotation(link['q']);np_=np.asarray(m['xyz'])+mr@np.asarray(link['xyz']);v=d['connectors'][0]['pose_6d'];arm=row['arms']['right_arm'];cmd=np.asarray(arm['command_tcp_pose']);tcp=np.asarray(arm['tcp']);tr=np.asarray(arm['tcp_rotation_world'])
 rows.append(dict(phase=phase,sensor_time_s=stamp,nearest_native_time_s=row['sim_time_s'],time_mismatch_s=abs(row['sim_time_s']-stamp),vision_vs_native_position_m=float(np.linalg.norm(np.asarray(v['center_xyz_m'])-np_)),vision_vs_native_angle_rad=float(np.linalg.norm(rotation_error(np.asarray(v['rotation_world_from_connector']),nr))),native_center_world=np_.tolist(),native_rotation_world=nr.tolist(),tcp_tracking_vector_m=(tcp-cmd[:3,3]).tolist(),tcp_tracking_position_m=float(np.linalg.norm(tcp-cmd[:3,3])),tcp_tracking_angle_rad=float(np.linalg.norm(rotation_error(tr,cmd[:3,:3]))),joint_tracking_error_rad=(np.asarray(arm['q_rad'])-arm['command_q_rad']).tolist(),fk_native_error_m=arm['fk_native_error_m']))
pts=json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0','-show_entries','packet=pts_time','-of','json',str(r/'raw.mkv')]))
times=[float(x['pts_time']) for x in pts['packets']]
report={'scope':'Post-trial nearest-time native evaluation only; no truth-driven targets or verdict override. Downsampled trajectory is not the complete feedback stream.','observations':rows,'native_center_displacement_m':float(np.linalg.norm(np.asarray(rows[-1]['native_center_world'])-rows[0]['native_center_world'])),'native_rotation_change_rad':float(np.linalg.norm(rotation_error(np.asarray(rows[-1]['native_rotation_world']),np.asarray(rows[0]['native_rotation_world'])))),'video_packet_timing':{'packets':len(times),'first_pts_s':times[0],'last_pts_s':times[-1],'maximum_gap_s':max(np.diff(times)),'gaps_over_0_4s':int(sum(np.diff(times)>.400001))}}
(r/'independent_perception_diagnostic.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
a=Path('docs/performance/20261030/full_dt0002/r02_failure');a.mkdir(exist_ok=False);manifest={}
for f in sorted(r.rglob('*')):
 if not f.is_file():continue
 rel=f.relative_to(r);data=f.read_bytes();sha=hashlib.sha256(data).hexdigest()
 if f.suffix=='.mkv':manifest[str(rel)]={'sha256_original':sha,'bytes':len(data),'local_path':str(f),'storage':'Original wall-clock video retained locally'};continue
 compressed=f.suffix in ['.npy','.log'] or (len(data)>200000 and f.suffix=='.json')
 dest=a/(str(rel)+('.gz' if compressed else ''));dest.parent.mkdir(parents=True,exist_ok=True);saved=gzip.compress(data,mtime=0) if compressed else data;dest.write_bytes(saved)
 assert (gzip.decompress(saved) if compressed else saved)==data
 manifest[str(rel)]={'sha256_original':sha,'sha256_archive':hashlib.sha256(saved).hexdigest(),'archive_path':str(dest.relative_to(a)),'encoding':'gzip-lossless' if compressed else 'unchanged'}
(a/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print('archived',len(manifest))
