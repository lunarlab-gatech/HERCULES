import math

from hercules_urdf_import.transforms import Transform
from hercules_urdf_import.urdf_model import parse_urdf_string, UrdfError
import pytest


def robot(body):
    return '<robot name="r">%s</robot>' % body


CHAIN = robot("""
  <link name="a"/><link name="b"/><link name="c"/>
  <joint name="ab" type="revolute">
    <parent link="a"/><child link="b"/>
    <origin xyz="1 0 0"/><axis xyz="0 0 1"/>
    <limit lower="0.5" upper="1.5"/>
  </joint>
  <joint name="bc" type="prismatic">
    <parent link="b"/><child link="c"/>
    <origin xyz="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-1" upper="1"/>
    <mimic joint="ab" multiplier="2" offset="0.5"/>
  </joint>
""")


def test_root_and_default_joint_positions():
    model = parse_urdf_string(CHAIN)
    assert model.root == 'a'
    positions = model.joint_positions()
    # 0 is outside [0.5, 1.5] so, like joint_state_publisher, use mid-range
    assert positions['ab'] == pytest.approx(1.0)
    assert positions['bc'] == pytest.approx(2.5)


def test_link_poses_follow_joint_motion():
    model = parse_urdf_string(CHAIN)
    poses = model.link_poses(model.joint_positions({'ab': math.pi / 2}))
    # c = a -> (1,0,0) -> rotate 90 deg about z -> slide (2*pi/2 + 0.5) along x
    slide = math.pi + 0.5
    expected = Transform.from_xyz_rpy((1, slide, 0), (0, 0, math.pi / 2))
    assert poses['c'].is_close(expected, 1e-9)


def test_overrides_must_name_real_joints():
    with pytest.raises(UrdfError, match='unknown joint'):
        parse_urdf_string(CHAIN).joint_positions({'nope': 1})


@pytest.mark.parametrize('body, message', [
    ('<link name="a"/><link name="b"/>', 'exactly one root'),
    ('<link name="a"/><link name="a"/>', 'duplicate link'),
    ('<link name="a"/><joint name="j" type="fixed"><parent link="a"/>'
     '<child link="zz"/></joint>', 'unknown link'),
    ('<link name="a"/><link name="b"/><joint name="j" type="hinge">'
     '<parent link="a"/><child link="b"/></joint>', 'invalid type'),
    ('<link name="a"><visual><geometry><capsule radius="1" length="1"/>'
     '</geometry></visual></link>', 'unsupported geometry'),
    ('<link name="a"><visual><origin xyz="1 2"/><geometry><sphere radius="1"/>'
     '</geometry></visual></link>', 'expected 3 numbers'),
    ('<link name="a"><visual><origin xyz="nan 0 0"/><geometry><sphere radius="1"/>'
     '</geometry></visual></link>', 'non-finite'),
    ('<link name="a"/><gazebo reference="ghost"><sensor type="imu" name="i"/>'
     '</gazebo>', 'unknown link'),
])
def test_malformed_urdf_is_rejected(body, message):
    with pytest.raises(UrdfError, match=message):
        parse_urdf_string(robot(body))


def test_cycles_are_rejected():
    text = robot("""<link name="a"/><link name="b"/><link name="c"/>
      <joint name="ab" type="fixed"><parent link="a"/><child link="b"/></joint>
      <joint name="bc" type="fixed"><parent link="b"/><child link="c"/></joint>
      <joint name="cb" type="fixed"><parent link="c"/><child link="b"/></joint>""")
    with pytest.raises(UrdfError):
        parse_urdf_string(text)


def test_unexpanded_xacro_is_reported():
    text = ('<robot name="r" xmlns:xacro="http://www.ros.org/wiki/xacro">'
            '<xacro:property name="w" value="1"/><link name="a"/></robot>')
    with pytest.raises(UrdfError, match='xacro'):
        parse_urdf_string(text)
    with pytest.raises(UrdfError, match='xacro'):
        parse_urdf_string(robot('<link name="${name}"/>'))


def test_materials_resolve_by_name():
    model = parse_urdf_string(robot("""
      <material name="blue"><color rgba="0 0 1 1"/></material>
      <link name="a"><visual><geometry><sphere radius="1"/></geometry>
        <material name="blue"/></visual></link>"""))
    assert model.links['a'].visuals[0].rgba == (0.0, 0.0, 1.0, 1.0)
