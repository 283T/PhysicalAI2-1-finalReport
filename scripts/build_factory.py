"""Factory prototype: articulated arms, conveyor carrier and a jointed flexible cable."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from build_world import child, plugin, box, link

ROOT = Path(__file__).resolve().parents[1]


def mass(element, value):
    element.find('inertial/mass').text = str(value)
    for item in element.findall('inertial/inertia/*'):
        item.text = str(value * .025**2 / 12)


def flexible_cable(model, count=12):
    body = model.find("link[@name='body']")
    for item in list(body):
        if item.tag == 'visual' and item.get('name', '').startswith('wire_'):
            body.remove(item)
    previous = 'body'
    length = .025
    for index in range(count):
        name = f'cable_{index}'
        segment = link(model, name)
        mass(segment, .004)
        # Link frames are at segment centers; chain extends from the plug along -Y.
        child(segment, 'pose', f'0 {- .015 - (index+.5)*length} .008 0 0 0')
        box(segment, 'insulation', f'.008 {length} .008', '0 0 0 0 0 0',
            '.8 .08 .04 1' if index % 2 == 0 else '.08 .08 .1 1')
        joint = child(model, 'joint', name=f'bend_{index}', type='universal')
        child(joint, 'parent', previous)
        child(joint, 'child', name)
        child(joint, 'pose', f'0 {length/2} 0 0 0 0')
        for tag, direction in (('axis','1 0 0'), ('axis2','0 0 1')):
            axis = child(joint, tag)
            child(axis, 'xyz', direction)
            dynamics = child(axis, 'dynamics')
            child(dynamics, 'damping', .002)
            child(dynamics, 'spring_reference', 0)
            child(dynamics, 'spring_stiffness', .003)
            limits = child(axis, 'limit')
            child(limits, 'lower', -1.2)
            child(limits, 'upper', 1.2)
            child(limits, 'effort', 1)
        previous = name
    # Report link poses for deformation verification; keep separate from plug-only pose topic.
    plugin(model, ('pose-publisher','PosePublisher'), publish_link_pose='true',
           publish_model_pose='false', use_pose_vector_msg='true',
           update_frequency=30, topic='/cable/poses')


def arm(world, name, origin, yaw, color):
    model = child(world, 'model', name=name)
    child(model, 'pose', f'{origin[0]} {origin[1]} .12 0 0 {yaw}')
    base = link(model, 'base', gravity=False)
    box(base, 'pedestal', '.09 .09 .15', '0 0 .075 0 0 0', color)
    fixed = child(model, 'joint', name='anchor', type='fixed')
    child(fixed, 'parent', 'world'); child(fixed, 'child', 'base')
    parent = 'base'
    for index, (center, size, pivot, axis) in enumerate((
        ((0,0,.20), '.06 .06 .10', (0,0,.15), '0 0 1'),
        ((.15,0,.25), '.30 .04 .04', (0,0,.25), '0 1 0'),
        ((.425,0,.25), '.25 .035 .035', (.30,0,.25), '0 1 0'),
        ((.59,0,.25), '.08 .04 .04', (.55,0,.25), '0 1 0'),
        ((.63,0,.25), '.035 .035 .035', (.63,0,.25), '1 0 0'),
        ((.63,0,.25), '.025 .025 .025', (.63,0,.25), '0 0 1'))):
        body = link(model, f'link_{index}', gravity=False)
        box(body, 'arm', size, ' '.join(map(str, (*center,0,0,0))), color)
        joint = child(model, 'joint', name=f'joint_{index}', type='revolute')
        child(joint,'parent',parent); child(joint,'child',f'link_{index}')
        child(joint,'pose', ' '.join(map(str, (*pivot,0,0,0))))
        ax=child(joint,'axis'); child(ax,'xyz',axis)
        lim=child(ax,'limit'); child(lim,'lower',-2.8); child(lim,'upper',2.8); child(lim,'effort',50)
        plugin(model, ('joint-position-controller','JointPositionController'),
               joint_name=f'joint_{index}', topic=f'/{name}/joint_{index}/cmd',
               p_gain=100, d_gain=10, cmd_max=50, cmd_min=-50)
        parent=f'link_{index}'
    plugin(model, ('joint-state-publisher','JointStatePublisher'), topic=f'/{name}/joints')
    for side in (-1,1):
        box(body, f'finger_{side}', '.02 .012 .08', f'.63 {side*.025} .21 0 0 0', color)


def main():
    tree=ET.parse(ROOT/'sim/worlds/insertion.sdf')
    world=tree.getroot().find('world'); world.set('name','harness_factory_prototype')
    world.remove(world.find("model[@name='gantry']"))
    fixture=world.find("model[@name='fixtures']/link")
    for item in list(fixture):
        if item.get('name','').startswith('supply_'):
            fixture.remove(item)
    harness=world.find("model[@name='wire_harness']")
    flexible_cable(harness)
    arm(world,'left_arm',(-.45,-.2),0,'.9 .4 .05 1')
    arm(world,'right_arm',(.45,-.2),math.pi,'.1 .35 .75 1')
    # A moving carrier approximates a conveyor pallet. Transport control is a separate stage.
    carrier=child(world,'model',name='conveyor_carrier')
    base=link(carrier,'carrier',gravity=False)
    box(base,'pallet','.12 .25 .02','-.25 -.08 .11 0 0 0','.35 .4 .45 1')
    joint=child(carrier,'joint',name='travel',type='prismatic')
    child(joint,'parent','world'); child(joint,'child','carrier')
    axis=child(joint,'axis'); child(axis,'xyz','1 0 0')
    lim=child(axis,'limit'); child(lim,'lower',-.15); child(lim,'upper',.3)
    plugin(carrier,('joint-position-controller','JointPositionController'),joint_name='travel',
           topic='/conveyor/cmd',p_gain=400,d_gain=40,cmd_max=20,cmd_min=-20)
    # Camera image is intended to feed perception, not simulator true object coordinates.
    camera=child(world,'model',name='overhead_camera'); child(camera,'static','true')
    camera_link=link(camera,'camera_link')
    child(camera_link,'pose','0 0 .95 0 1.57079632679 0')
    sensor=child(camera_link,'sensor',name='rgb',type='camera')
    child(sensor,'always_on','true'); child(sensor,'update_rate',10); child(sensor,'topic','/factory/image')
    cam=child(sensor,'camera'); child(cam,'horizontal_fov',1.4)
    im=child(cam,'image'); child(im,'width',640); child(im,'height',480); child(im,'format','R8G8B8')
    clip=child(cam,'clip'); child(clip,'near',.05); child(clip,'far',3)
    plugin(world,('sensors','Sensors'),render_engine='ogre')
    ET.indent(tree)
    tree.write(ROOT/'sim/worlds/factory_prototype.sdf',encoding='utf-8',xml_declaration=True)
    # Independent cable test: anchor plug and let the cable droop freely beyond the table edge.
    test=ET.Element('sdf',version='1.9'); scene=child(test,'world',name='cable_validation')
    physics=child(scene,'physics',name='physics',type='ignored'); child(physics,'max_step_size',.001)
    child(scene,'gravity','0 0 -9.81')
    plugin(scene,('physics','Physics')); plugin(scene,('scene-broadcaster','SceneBroadcaster'))
    import copy
    cable=copy.deepcopy(harness); cable.find('pose').text='0 0 .45 0 0 0'
    anchor=child(cable,'joint',name='test_anchor',type='fixed'); child(anchor,'parent','world'); child(anchor,'child','body')
    scene.append(cable)
    ET.indent(test); ET.ElementTree(test).write(ROOT/'sim/worlds/cable_test.sdf',encoding='utf-8',xml_declaration=True)


if __name__=='__main__':
    main()
