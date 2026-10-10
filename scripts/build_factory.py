"""Factory prototype: articulated arms, conveyor carrier and a jointed flexible cable."""

import math
import copy
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


JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')


def arm(world, name, origin, yaw):
    """Load the official UR3e geometry, joint frames, limits and inertias."""
    import subprocess
    import tempfile
    from ament_index_python.packages import get_package_share_directory
    description = Path(get_package_share_directory('ur_description'))
    prefix = name + '_'
    urdf = subprocess.check_output([
        'xacro', str(description/'urdf/ur.urdf.xacro'), 'ur_type:=ur3e',
        f'name:={name}', f'tf_prefix:={prefix}', 'force_abs_paths:=true'], text=True)
    robot = ET.fromstring(urdf)
    # Simple fixed fingers are only a placeholder tool, not an actuated gripper.
    tool = child(robot, 'link', name=prefix+'gripper')
    inertial = child(tool, 'inertial')
    child(inertial, 'mass', value='.05')
    child(inertial, 'inertia', ixx='.00002', iyy='.00002', izz='.00002',
          ixy='0', ixz='0', iyz='0')
    for side in (-1, 1):
        for kind in ('visual', 'collision'):
            shape = child(tool, kind, name=f'finger_{side}')
            child(shape, 'origin', xyz=f'{side*.025} 0 .04', rpy='0 0 0')
            child(child(shape, 'geometry'), 'box', size='.012 .02 .08')
    fixed = child(robot, 'joint', name=prefix+'gripper_mount', type='fixed')
    child(fixed, 'parent', link=prefix+'tool0')
    child(fixed, 'child', link=prefix+'gripper')
    with tempfile.TemporaryDirectory() as directory:
        source = Path(directory)/'arm.urdf'
        ET.ElementTree(robot).write(source, encoding='utf-8', xml_declaration=True)
        converted = subprocess.check_output(['gz', 'sdf', '-p', str(source)], text=True)
    model = ET.fromstring(converted).find('model')
    if model is None:
        raise RuntimeError('UR3e conversion produced no model')
    model.set('name', name)
    child(model, 'pose', f'{origin[0]} {origin[1]} .12 0 0 {yaw}')
    for index, joint in enumerate(JOINTS):
        effort = float(model.find(f"joint[@name='{prefix+joint}']/axis/limit/effort").text)
        plugin(model, ('joint-position-controller','JointPositionController'),
               joint_name=prefix+joint, topic=f'/{name}/joint_{index}/cmd',
               p_gain=800 if index < 3 else 80, d_gain=8 if index < 3 else .1,
               i_gain=100 if index < 3 else 20, i_max=effort, i_min=-effort,
               cmd_max=effort, cmd_min=-effort)
    plugin(model, ('joint-state-publisher','JointStatePublisher'), topic=f'/{name}/joints')
    world.append(model)


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
    arm(world,'left_arm',(-.15,-.28),math.pi)
    arm(world,'right_arm',(.38,-.28),0)
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
    # Keep motor validation independent of cable contact and camera rendering.
    arm_test=ET.Element('sdf', version='1.9')
    arm_world=child(arm_test, 'world', name='dual_ur3e_validation')
    arm_physics=child(arm_world, 'physics', name='physics', type='ignored')
    child(arm_physics, 'max_step_size', .001)
    child(arm_world, 'gravity', '0 0 -9.81')
    plugin(arm_world, ('physics','Physics'))
    plugin(arm_world, ('scene-broadcaster','SceneBroadcaster'))
    for name in ('left_arm', 'right_arm'):
        arm_world.append(copy.deepcopy(world.find(f"model[@name='{name}']")))
    ET.indent(arm_test)
    ET.ElementTree(arm_test).write(ROOT/'sim/worlds/arms_test.sdf', encoding='utf-8', xml_declaration=True)
    # Independent cable test: anchor plug and let the cable droop freely beyond the table edge.
    test=ET.Element('sdf',version='1.9'); scene=child(test,'world',name='cable_validation')
    physics=child(scene,'physics',name='physics',type='ignored'); child(physics,'max_step_size',.001)
    child(scene,'gravity','0 0 -9.81')
    plugin(scene,('physics','Physics')); plugin(scene,('scene-broadcaster','SceneBroadcaster'))
    cable=copy.deepcopy(harness); cable.find('pose').text='0 0 .45 0 0 0'
    anchor=child(cable,'joint',name='test_anchor',type='fixed'); child(anchor,'parent','world'); child(anchor,'child','body')
    scene.append(cable)
    ET.indent(test); ET.ElementTree(test).write(ROOT/'sim/worlds/cable_test.sdf',encoding='utf-8',xml_declaration=True)


if __name__=='__main__':
    main()
