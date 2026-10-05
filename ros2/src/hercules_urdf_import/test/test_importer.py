import json
import math
import os
import shutil
from urllib.parse import unquote, urlparse
from urllib.request import url2pathname

from conftest import DESCRIPTION
from hercules_urdf_import import import_robot, ImportOptions
from hercules_urdf_import.cli import main
from hercules_urdf_import.transforms import OPTICAL_FROM_BODY, quat_from_rpy, Transform
from hercules_urdf_import.urdf_model import parse_urdf_string, UrdfError
import pytest

URDF = os.path.join(DESCRIPTION, 'urdf')
CONFIG = os.path.join(DESCRIPTION, 'config')
HERCULES_ROOT = os.path.abspath(os.path.join(DESCRIPTION, *(['..'] * 6)))


def run(tmp_path, urdf, name, drive, **kwargs):
    return import_robot(ImportOptions(os.path.join(URDF, urdf), name, drive,
                                      str(tmp_path), **kwargs))


def wrapper_tf(entry):
    """
    Replay hercules_ros_wrapper's TF for a settings pose.

    The wrapper applies tf2 setRPY and then convert_tf_msg_to_ros; the result
    is relative to <vehicle>/ground_truth/odom_local.
    """
    w, x, y, z = quat_from_rpy(*(math.radians(entry[k]) for k in ('Roll', 'Pitch', 'Yaw')))
    return Transform((w, x, -y, -z), (entry['X'], -entry['Y'], -entry['Z']))


def frame_in_body(result, frame, odom_frame='odom_local'):
    """Pose of `frame` in the body frame, read from the generated URDF."""
    model = parse_urdf_string(result.urdf)
    assert model.root == 'ground_truth/' + odom_frame
    return model.link_poses()[frame]


def test_skid_robot_settings_and_frames(tmp_path):
    result = run(tmp_path, 'skid_robot.urdf', 'Husky3', 'skid')
    settings = result.settings
    assert settings['SimMode'] == 'Hero'
    assert 'UGVPawn' in settings['PawnPaths']
    vehicle = settings['Vehicles']['Husky3']
    assert vehicle['VehicleType'] == 'PhysXCar'
    assert vehicle['PawnPath'] == 'UGVPawn'
    assert vehicle['Urdf']['HideBaseMesh'] is True
    assert vehicle['Urdf']['TwoSided'] is False
    # the imu has no frame of its own: its data is in the body frame
    assert vehicle['Urdf']['SensorFrames'] == ['front_cam', 'os1_64', 'sonar_front']
    assert os.path.isfile(vehicle['Urdf']['Visuals'])
    assert os.path.isfile(vehicle['Urdf']['RobotDescription'])

    # base_footprint (ground level) is the body frame for wheeled robots
    camera = vehicle['Cameras']['front_cam']
    assert camera['X'] == pytest.approx(0.45)
    assert camera['Y'] == pytest.approx(0.1)          # 0.1 m right
    assert camera['Z'] == pytest.approx(-0.33)        # 0.13 + 0.2 m up
    assert camera['Pitch'] == pytest.approx(-math.degrees(0.2), abs=1e-5)
    assert camera['Yaw'] == pytest.approx(math.degrees(0.1), abs=1e-5)
    assert camera['CaptureSettings'] == [
        {'ImageType': 0, 'Width': 1280, 'Height': 720, 'FOV_Degrees': 90.0}]

    lidar = vehicle['Sensors']['os1_64']
    assert lidar['SensorType'] == 8
    assert lidar['NumberOfChannels'] == 64
    assert lidar['MeasurementsPerCycle'] == 1024
    assert lidar['VerticalFOVUpper'] == pytest.approx(22.5)
    assert lidar['VerticalFOVLower'] == pytest.approx(-22.5)
    assert 'HorizontalFOVStart' not in lidar  # full circle keeps the defaults
    assert lidar['Z'] == pytest.approx(-(0.13 + 0.3 + 0.036))

    assert vehicle['Sensors']['imu'] == {'SensorType': 2, 'Enabled': True}
    imu = next(s for s in result.manifest['sensors'] if s['name'] == 'imu')
    assert imu['link'] == 'imu_link' and imu['frd_pose'] is None
    assert vehicle['Sensors']['sonar_front']['SensorType'] == 5
    assert vehicle['Sensors']['sonar_front']['MaxDistance'] == 4.0
    assert any('bumper_contact' in w for w in result.warnings)
    assert any('imu "imu" is offset' in w for w in result.warnings)


def test_urdf_frames_match_what_the_wrapper_would_publish(tmp_path):
    result = run(tmp_path, 'skid_robot.urdf', 'Husky3', 'skid')
    vehicle = result.vehicle
    camera = vehicle['Cameras']['front_cam']
    body = frame_in_body(result, 'front_cam_body')
    assert body.is_close(wrapper_tf(camera), 1e-5)
    optical = frame_in_body(result, 'front_cam_optical')
    assert optical.is_close(wrapper_tf(camera) * OPTICAL_FROM_BODY, 1e-5)
    # the URDF's own optical frame is the same frame as ours
    assert optical.is_close(frame_in_body(result, 'camera_link_optical'), 1e-9)
    for name in ('os1_64', 'sonar_front'):
        assert frame_in_body(result, name).is_close(
            wrapper_tf(vehicle['Sensors'][name]), 1e-5)


def test_output_urdf_is_static_and_rooted_at_the_body(tmp_path):
    result = run(tmp_path, 'skid_robot.urdf', 'Husky3', 'skid')
    model = parse_urdf_string(result.urdf)
    assert all(j.type == 'fixed' for j in model.joints.values())
    assert model.link_poses()['base_footprint'].is_close(Transform(), 1e-12)
    for mesh in model.links['base_link'].visuals[1:]:
        uri = mesh.geometry.filename
        assert uri.startswith('file:///')
        assert os.path.isfile(url2pathname(unquote(urlparse(uri).path)))


def test_visual_manifest(tmp_path):
    result = run(tmp_path, 'skid_robot.urdf', 'Husky3', 'skid')
    manifest = result.manifest
    assert manifest['format'] == 'hercules-urdf-visuals'
    assert manifest['frame'] == 'base_footprint'
    by_name = {v['name']: v for v in manifest['visuals']}
    # STL keeps the URDF material, Collada brings its own color
    assert by_name['top_plate']['color'] == [1.0, 0.8, 0.0, 1.0]
    assert by_name['front_bumper']['color'] == [0.8, 0.1, 0.1, 1.0]
    assert by_name['front_bumper']['position'] == pytest.approx([0.5, 0.0, 0.13])
    # the OBJ's two materials become two visuals
    assert {'mast_link_visual_0_0', 'mast_link_visual_0_1'} <= set(by_name)
    # four wheels share one mesh file
    wheels = [v for v in manifest['visuals'] if v['link'].endswith('_wheel')]
    assert len(wheels) == 4 and len({v['mesh'] for v in wheels}) == 1
    for visual in manifest['visuals']:
        assert os.path.isfile(os.path.join(str(tmp_path), visual['mesh']))
    on_disk = json.loads((tmp_path / 'Husky3.visuals.json').read_text())
    assert on_disk == json.loads(json.dumps(manifest))


def test_ackermann_with_sensor_config_and_optical_link(tmp_path):
    result = run(tmp_path, 'ackermann_car.urdf', 'Car1', 'ackermann',
                 sensor_config=os.path.join(CONFIG, 'ackermann_sensors.yaml'),
                 sim_mode='native')
    assert result.settings['SimMode'] == 'Car'
    vehicle = result.vehicle
    assert vehicle['VehicleType'] == 'PhysXCar' and 'PawnPath' not in vehicle
    camera = vehicle['Cameras']['front_center']
    # an optical link looking forward and 10 degrees down
    assert camera['Roll'] == pytest.approx(0.0, abs=1e-5)
    assert camera['Pitch'] == pytest.approx(-10.0, abs=1e-5)
    assert camera['Yaw'] == pytest.approx(0.0, abs=1e-5)
    assert [c['ImageType'] for c in camera['CaptureSettings']] == [0, 1, 5]
    lidar = vehicle['Sensors']['roof_lidar']
    assert lidar['SensorType'] == 6
    # ROS [-60, 60] (CCW) is the same window in the sim's clockwise azimuth
    assert (lidar['HorizontalFOVStart'], lidar['HorizontalFOVEnd']) == (-60.0, 60.0)
    assert lidar['VerticalFOVUpper'] == 10.0 and lidar['VerticalFOVLower'] == -15.0
    assert vehicle['Sensors']['gps'] == {'SensorType': 3, 'Enabled': True}
    # the steering mimic joint froze with its leader
    model = parse_urdf_string(result.urdf)
    assert all(j.type == 'fixed' for j in model.joints.values())


def test_asymmetric_lidar_window_is_mirrored(tmp_path):
    config = {'lidars': [{'name': 'side', 'link': 'roof',
                          'horizontal_fov_deg': [10, 80]}]}
    result = run(tmp_path, 'ackermann_car.urdf', 'Car1', 'ackermann', sensor_config=config)
    lidar = result.vehicle['Sensors']['side']
    assert (lidar['HorizontalFOVStart'], lidar['HorizontalFOVEnd']) == (-80.0, -10.0)


def test_quadrotor_gimbal_joint_override(tmp_path):
    result = run(tmp_path, 'quadrotor.urdf', 'Drone3', 'multirotor',
                 joint_positions={'gimbal_pitch': 0.3, 'gimbal_yaw': -0.2})
    assert result.vehicle['VehicleType'] == 'SimpleFlight'
    camera = result.vehicle['Cameras']['gimbal_cam']
    assert camera['Pitch'] == pytest.approx(-math.degrees(0.3), abs=1e-5)
    assert camera['Yaw'] == pytest.approx(math.degrees(0.2), abs=1e-5)
    # 6 + 2 cm below base_link, plus the 3 cm lens offset pitched down 0.3 rad
    assert camera['Z'] == pytest.approx(0.08 + 0.03 * math.sin(0.3), abs=1e-6)
    assert any('gimbal_cam' in w and 'gimbal_pitch' in w for w in result.warnings)
    assert frame_in_body(result, 'gimbal_cam_body').is_close(wrapper_tf(camera), 1e-5)


def test_articulated_robot_defaults_and_keep_joints(tmp_path):
    result = run(tmp_path, 'mobile_manipulator.urdf', 'Arm1', 'skid', freeze_joints=False)
    camera = result.vehicle['Cameras']['wrist_cam']
    assert [c['ImageType'] for c in camera['CaptureSettings']] == [0, 1]
    # elbow defaults to mid-range 1.5 rad, the wrist mimics it: -1.5 + 0.1
    pitch_total = 1.5 + (-1.5 + 0.1)
    assert camera['Pitch'] == pytest.approx(-math.degrees(pitch_total), abs=1e-4)
    model = parse_urdf_string(result.urdf)
    movable = sorted(j.name for j in model.joints.values() if j.movable)
    assert movable == ['elbow', 'finger_joint', 'shoulder', 'wrist']


def test_merge_into_hero_team_settings(tmp_path):
    team = os.path.join(HERCULES_ROOT, 'ros2', 'settings', 'hero_blocks_team.json')
    if not os.path.isfile(team):
        pytest.skip('not running inside the HERCULES repository')
    copy = tmp_path / 'team.json'
    shutil.copy(team, copy)
    result = run(tmp_path, 'skid_robot.urdf', 'Husky3', 'skid', merge_settings=str(copy),
                 position=(6.0, 6.0, 0.0), yaw_deg=45.0)
    vehicles = result.settings['Vehicles']
    assert {'Drone1', 'Husky1', 'Husky3'} <= set(vehicles)
    assert vehicles['Husky3']['X'] == 6.0 and vehicles['Husky3']['Yaw'] == 45.0
    assert json.loads(copy.read_text()) == json.load(open(team))  # input untouched


def test_relative_paths(tmp_path):
    result = run(tmp_path, 'skid_robot.urdf', 'Husky3', 'skid', relative_paths=True)
    assert result.vehicle['Urdf']['Visuals'] == 'Husky3.visuals.json'
    assert result.vehicle['Urdf']['RobotDescription'] == 'Husky3.urdf'


@pytest.mark.parametrize('kwargs, message', [
    ({'drive': 'static'}, 'cannot run in the hero sim mode'),
    ({'vehicle_name': '3bot'}, 'must start with a letter'),
    ({'vehicle_name': 'my-bot'}, 'must start with a letter'),
    ({'drive': 'hover'}, 'drive must be one of'),
    ({'base_link': 'nope'}, 'not in the URDF'),
])
def test_bad_options(tmp_path, kwargs, message):
    args = {'urdf_path': os.path.join(URDF, 'skid_robot.urdf'), 'vehicle_name': 'Bot',
            'drive': 'skid', 'output_dir': str(tmp_path)}
    args.update(kwargs)
    with pytest.raises(UrdfError, match=message):
        import_robot(ImportOptions(**args))


def test_gpu_lidar_is_refused_in_multirotor_mode(tmp_path):
    with pytest.raises(UrdfError, match='GPU lidar'):
        run(tmp_path, 'skid_robot.urdf', 'Bot', 'multirotor', sim_mode='native')


@pytest.mark.parametrize('config, frame', [
    # a camera named "camera_link" would add camera_link_optical a second time
    ({'cameras': [{'name': 'camera_link', 'link': 'base_link'}]}, 'camera_link_optical'),
    ({'lidars': [{'name': 'mast_link', 'link': 'base_link'}]}, 'mast_link'),
])
def test_sensor_frame_name_clash(tmp_path, config, frame):
    with pytest.raises(UrdfError, match='already has a link named "%s"' % frame):
        run(tmp_path, 'skid_robot.urdf', 'Bot', 'skid', sensor_config=config)


@pytest.mark.parametrize('config, message', [
    ({'radars': []}, 'unknown sensor config section'),
    ({'cameras': [{'link': 'base_link'}]}, 'needs a name'),
    ({'cameras': [{'name': 'c'}]}, 'needs the URDF link'),
    ({'cameras': [{'name': 'c', 'link': 'ghost'}]}, 'unknown link'),
    ({'cameras': [{'name': 'c', 'link': 'base_link', 'zoom': 2}]}, 'unknown key'),
    ({'cameras': [{'name': 'c', 'link': 'base_link', 'frame': 'cv'}]}, 'frame must be'),
    ({'cameras': [{'name': 'c', 'link': 'base_link', 'image_types': ['Lidar']}]},
     'unknown image type'),
    ({'cameras': [{'name': 'c', 'link': 'base_link', 'fov_deg': 180}]}, 'fov_deg'),
    ({'lidars': [{'name': 'l', 'link': 'base_link', 'rotations_per_second': 7.5}]},
     'whole number'),
    ({'lidars': [{'name': 'l', 'link': 'base_link', 'horizontal_fov_deg': [5, -5]}]},
     r'\[min, max\]'),
    ({'distances': [{'name': 'd', 'link': 'base_link', 'min': 3, 'max': 1}]},
     'min must be below max'),
])
def test_bad_sensor_config(tmp_path, config, message):
    with pytest.raises(UrdfError, match=message):
        run(tmp_path, 'skid_robot.urdf', 'Bot', 'skid', sensor_config=config)


def test_config_overrides_and_disables_gazebo_sensors(tmp_path):
    config = {'cameras': [{'name': 'front_cam', 'width': 640, 'height': 480,
                           'link': 'camera_link'}],
              'distances': [{'name': 'sonar_front', 'enabled': False}]}
    result = run(tmp_path, 'skid_robot.urdf', 'Bot', 'skid', sensor_config=config)
    capture = result.vehicle['Cameras']['front_cam']['CaptureSettings'][0]
    assert (capture['Width'], capture['Height'], capture['FOV_Degrees']) == (640, 480, 90.0)
    assert 'sonar_front' not in result.vehicle['Sensors']


def test_missing_mesh_is_a_warning_not_an_error(tmp_path):
    urdf = tmp_path / 'r.urdf'
    urdf.write_text('<robot name="r"><link name="base_link"><visual><geometry>'
                    '<mesh filename="package://nowhere_pkg/m.stl"/></geometry>'
                    '</visual></link></robot>')
    result = import_robot(ImportOptions(str(urdf), 'Bot', 'skid', str(tmp_path / 'out')))
    assert result.manifest['visuals'] == []
    assert any('nowhere_pkg' in w for w in result.warnings)


def test_cli(tmp_path, capsys):
    rc = main([os.path.join(URDF, 'quadrotor.urdf'), '--name', 'Drone3', '--drive',
               'multirotor', '-o', str(tmp_path), '--joint', 'gimbal_pitch=0.3',
               '--yaw', '90', '--two-sided', '--show-base-mesh'])
    assert rc == 0
    out = capsys.readouterr()
    assert 'Imported "Drone3" as SimpleFlight' in out.out
    settings = json.loads((tmp_path / 'Drone3.settings.json').read_text())
    assert settings['Vehicles']['Drone3']['Yaw'] == 90.0
    assert settings['Vehicles']['Drone3']['Urdf']['TwoSided'] is True
    assert settings['Vehicles']['Drone3']['Urdf']['HideBaseMesh'] is False
    assert main([os.path.join(URDF, 'quadrotor.urdf'), '--name', 'D', '--drive',
                 'multirotor', '-o', str(tmp_path), '--joint', 'gimbal_pitch']) == 1
    assert 'NAME=VALUE' in capsys.readouterr().err


def test_xacro_input(tmp_path):
    pytest.importorskip('xacro')
    source = tmp_path / 'bot.urdf.xacro'
    source.write_text("""<?xml version="1.0"?>
<robot name="bot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="size" default="0.5"/>
  <xacro:property name="s" value="$(arg size)"/>
  <link name="base_link"><visual><geometry><box size="${s} ${s} ${s}"/></geometry></visual></link>
</robot>""")
    result = import_robot(ImportOptions(str(source), 'Bot', 'skid', str(tmp_path / 'out'),
                                        xacro_args={'size': '0.25'}))
    assert len(result.manifest['visuals']) == 1
    assert 'size="0.25 0.25 0.25"' in result.urdf
