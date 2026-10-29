"""Prepare conservative arm-mesh collision boxes for a separate throughput experiment."""
import argparse
import copy
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from itertools import product
from pathlib import Path

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
import numpy as np
from arm_collision_bounds import mesh_bounds
from fixture_collision_bounds import sdf_pose


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--source',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    if args.output.exists():parser.error('Preserve previous experiments')
    args.output.mkdir(parents=True);tree=ET.parse(args.source);world=tree.getroot().find('world');changes=[]
    for model in world.findall('model'):
        if model.get('name') not in ['left_arm','right_arm']:continue
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                geometry=collision.find('geometry');mesh=geometry.find('mesh')
                if mesh is None:continue
                original=copy.deepcopy(collision);pose=sdf_pose(collision);scale=np.array([float(v) for v in mesh.findtext('scale','1 1 1').split()]);low,high=mesh_bounds(mesh.findtext('uri'),scale);size=high-low;center=(high+low)/2
                if min(size)<=0:raise ValueError('Degenerate mesh bounds')
                values=[float(v) for v in collision.findtext('pose','0 0 0 0 0 0').split()];values[:3]=(pose[:3,3]+pose[:3,:3]@center).tolist()
                field=collision.find('pose')
                if field is None:field=ET.SubElement(collision,'pose')
                field.text=' '.join(format(v,'.17g') for v in values)
                geometry.remove(mesh);box=ET.SubElement(geometry,'box');ET.SubElement(box,'size').text=' '.join(format(v,'.17g') for v in size)
                actual=sdf_pose(collision);inverse=np.linalg.inv(actual);maximum_excess=0.
                for corner in product(*zip(low,high)):
                    point=pose@np.r_[corner,1];local=(inverse@point)[:3];maximum_excess=max(maximum_excess,float(np.max(np.abs(local)-size/2)))
                assert maximum_excess<1e-12
                # All collision properties except pose/geometry, and all visuals,
                # inertias, controllers and physical thresholds are preserved.
                for element in [original,collision]:
                    for tag in ['pose','geometry']:
                        child=element.find(tag)
                        if child is not None:element.remove(child)
                assert ET.tostring(original)==ET.tostring(collision)
                # Restore the prepared geometry and pose after the property check.
                collision.append(field);collision.append(geometry)
                changes.append({'model':model.get('name'),'link':link.get('name'),'collision':collision.get('name'),'mesh_uri':mesh.findtext('uri'),'low_local_m':low.tolist(),'high_local_m':high.tolist(),'box_size_m':size.tolist(),'box_pose':values,'aabb_corner_containment_maximum_excess_m':maximum_excess})
    target=args.output/'world.sdf';tree.write(target,encoding='utf-8',xml_declaration=True)
    assert len(changes)==14
    report={'scope':'Diagnostic conservative collision approximation, not a geometry-equivalent physical model or successful pickup; boxes can introduce additional contacts','source_sha256':hashlib.sha256(args.source.read_bytes()).hexdigest(),'world_sha256':hashlib.sha256(target.read_bytes()).hexdigest(),'physics_step_s':float(world.findtext('physics/max_step_size')),'arm_mesh_collisions_replaced':len(changes),'visuals_inertias_controller_limits_and_sensors':'unchanged','collision_names':'preserved','source_scaled_mesh_aabbs_contained':True,'changes':changes,'adoption':'Requires separate full correctness/contact validation; never substituted into formal B'}
    (args.output/'variant.json').write_text(json.dumps(report,indent=2)+'\n');print('Prepared',len(changes),'conservative arm collision boxes')

if __name__=='__main__':main()
