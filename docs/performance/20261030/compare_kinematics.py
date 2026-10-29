"""Compare optimized FK/Jacobians with the preserved original and time both in alternating order."""
import argparse
import hashlib
import importlib.util
import json
import platform
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
import numpy as np
from arm_kinematics import ArmKinematics


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    if args.output.exists():parser.error('Preserve previous comparisons')
    path=Path(__file__).with_name('reference_arm_kinematics.py');spec=importlib.util.spec_from_file_location('reference_fk',path);reference=importlib.util.module_from_spec(spec);spec.loader.exec_module(reference)
    world=ET.parse(ROOT/'outputs/stepped_nest_contacts_hd_20261016/case/world.sdf').getroot().find('world')
    rng=np.random.default_rng(20261030);pairs=[];maximum_pose=maximum_jacobian=0.;cases=0
    for name in ['left_arm','right_arm']:
        base=[float(v) for v in world.findtext(f"model[@name='{name}']/pose").split()];urdf=ROOT/f'outputs/stepped_hd_shm64_20261016/models/{name}.urdf'
        old=reference.ArmKinematics(urdf,name+'_gripper',base);new=ArmKinematics(urdf,name+'_gripper',base)
        vectors=np.vstack([np.zeros(6),old.limits[:,0],old.limits[:,1],rng.uniform(old.limits[:,0],old.limits[:,1],(1000,6))])
        for q in vectors:
            a,aj=old.forward(q,jacobian=True);b,bj=new.forward(q,jacobian=True);bn=new.forward(q)
            maximum_pose=max(maximum_pose,float(np.max(np.abs(a-b))),float(np.max(np.abs(b-bn))));maximum_jacobian=max(maximum_jacobian,float(np.max(np.abs(aj-bj))));np.testing.assert_allclose(a,b,atol=1e-12,rtol=0);np.testing.assert_allclose(aj,bj,atol=1e-12,rtol=0);cases+=1
        pairs.append((old,new,vectors))
    rows=[]
    for round_id,order in enumerate([['reference','candidate'],['candidate','reference'],['reference','candidate']]):
        row={'round':round_id,'order':order}
        for label in order:
            start=time.perf_counter();count=0
            for old,new,vectors in pairs:
                arm=old if label=='reference' else new
                for _ in range(3):
                    for q in vectors:arm.forward(q);count+=1
            row[label+'_s']=time.perf_counter()-start;row['calls_per_implementation']=count
        row['speedup']=row['reference_s']/row['candidate_s'];rows.append(row)
    report={'scope':'Numerical FK/Jacobian equivalence and isolated FK throughput; not physics/grasp/full task validation','platform':platform.platform(),'python':sys.version,'numpy':np.__version__,'reference_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'candidate_sha256':hashlib.sha256((ROOT/'scripts/arm_kinematics.py').read_bytes()).hexdigest(),'random_seed':20261030,'pose_jacobian_cases':cases,'maximum_absolute_pose_difference':maximum_pose,'maximum_absolute_jacobian_difference':maximum_jacobian,'tolerance':1e-12,'alternating_timing_rounds':rows,'numerical_checks_passed':True}
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))

if __name__=='__main__':main()
