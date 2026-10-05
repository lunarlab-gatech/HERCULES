import json
import os
import shutil

from conftest import FIXTURES
from hercules_urdf_import import import_robot, ImportOptions
from hercules_urdf_import.ouster import read_ouster_metadata
from hercules_urdf_import.urdf_model import UrdfError
import pytest

OUSTER = os.path.join(FIXTURES, 'ouster')
SKID = os.path.join(FIXTURES, 'test_robot_description', 'urdf', 'skid_robot.urdf')


@pytest.mark.parametrize('name, columns, rate', [
    ('os1_16_synthetic.json', 1024, 10),
    ('os1_16_synthetic_legacy.json', 512, 20),
])
def test_both_metadata_layouts(name, columns, rate):
    beams = read_ouster_metadata(os.path.join(OUSTER, name))
    assert beams['channels'] == 16
    assert beams['vertical_angles'][0] == 15.379 and beams['vertical_angles'][-1] == -15.703
    assert beams['azimuth_offsets'][:4] == [3.12, 0.92, -1.32, -3.54]
    assert (beams['measurements_per_cycle'], beams['rotations_per_second']) == (columns, rate)
    assert beams['product'] == 'OS-1-16'


@pytest.mark.parametrize('change, message', [
    (lambda d: d['beam_intrinsics'].pop('beam_altitude_angles'), 'beam_altitude_angles'),
    (lambda d: d['beam_intrinsics']['beam_azimuth_angles'].pop(), '16 beam altitudes but 15'),
    (lambda d: d['config_params'].update(lidar_mode='fast'), 'lidar_mode'),
    (lambda d: d['lidar_data_format'].update(columns_per_frame=2048), 'columns_per_frame'),
    (lambda d: d['lidar_data_format'].update(pixels_per_column=32), 'pixels_per_column'),
    (lambda d: d['beam_intrinsics']['beam_altitude_angles'].__setitem__(0, 'x'), 'numbers'),
])
def test_bad_metadata(tmp_path, change, message):
    with open(os.path.join(OUSTER, 'os1_16_synthetic.json')) as stream:
        data = json.load(stream)
    change(data)
    path = tmp_path / 'meta.json'
    path.write_text(json.dumps(data))
    with pytest.raises(UrdfError, match=message):
        read_ouster_metadata(str(path))


def test_missing_azimuths_default_to_zero(tmp_path):
    with open(os.path.join(OUSTER, 'os1_16_synthetic.json')) as stream:
        data = json.load(stream)
    del data['beam_intrinsics']['beam_azimuth_angles']
    path = tmp_path / 'meta.json'
    path.write_text(json.dumps(data))
    assert read_ouster_metadata(str(path))['azimuth_offsets'] == [0.0] * 16


def test_ouster_lidar_from_metadata(tmp_path):
    # paths in a sensor file are relative to that file
    config = tmp_path / 'sensors.json'
    shutil.copy(os.path.join(OUSTER, 'os1_16_synthetic.json'), str(tmp_path / 'os1.json'))
    config.write_text(json.dumps({'lidars': [
        {'name': 'os1', 'link': 'os_sensor', 'ouster_metadata': 'os1.json', 'range': 120}]}))
    result = import_robot(ImportOptions(SKID, 'Bot', 'skid', str(tmp_path / 'out'),
                                        sensor_config=str(config), gazebo_sensors=False))
    lidar = result.vehicle['Sensors']['os1']
    assert lidar['SensorType'] == 6
    assert lidar['NumberOfChannels'] == 16
    assert lidar['VerticalAngles'][0] == 15.379 and len(lidar['VerticalAngles']) == 16
    assert lidar['AzimuthOffsets'][:4] == [3.12, 0.92, -1.32, -3.54]
    assert (lidar['VerticalFOVUpper'], lidar['VerticalFOVLower']) == (15.379, -15.703)
    assert (lidar['MeasurementsPerCycle'], lidar['RotationsPerSecond'], lidar['Range']) == \
        (1024, 10, 120.0)


def test_config_overrides_metadata_rate(tmp_path):
    config = {'lidars': [{'name': 'os1', 'link': 'os_sensor', 'rotations_per_second': 20,
                          'ouster_metadata': os.path.join(OUSTER, 'os1_16_synthetic.json')}]}
    result = import_robot(ImportOptions(SKID, 'Bot', 'skid', str(tmp_path), sensor_config=config,
                                        gazebo_sensors=False))
    assert result.vehicle['Sensors']['os1']['RotationsPerSecond'] == 20


def test_channel_layout_cannot_contradict_metadata(tmp_path):
    config = {'lidars': [{'name': 'os1', 'link': 'os_sensor', 'channels': 64,
                          'ouster_metadata': os.path.join(OUSTER, 'os1_16_synthetic.json')}]}
    with pytest.raises(UrdfError, match='comes from ouster_metadata'):
        import_robot(ImportOptions(SKID, 'Bot', 'skid', str(tmp_path), sensor_config=config,
                                   gazebo_sensors=False))


def test_gpu_lidar_approximates_the_beam_table(tmp_path):
    config = {'gpulidars': [{'name': 'os1', 'link': 'os_sensor',
                             'ouster_metadata': os.path.join(OUSTER, 'os1_16_synthetic.json')}]}
    result = import_robot(ImportOptions(SKID, 'Bot', 'skid', str(tmp_path), sensor_config=config,
                                        gazebo_sensors=False))
    lidar = result.vehicle['Sensors']['os1']
    assert lidar['SensorType'] == 8 and 'VerticalAngles' not in lidar
    assert (lidar['VerticalFOVUpper'], lidar['NumberOfChannels']) == (15.379, 16)
    assert any('approximated' in w for w in result.warnings)
