"""
Map URDF/Gazebo sensors and a sensor config file onto HERCULES sensors.

Sensors come from two places, merged by name with the config file winning:

* ``<gazebo reference="link"><sensor type=...>`` blocks in the URDF (both
  Gazebo Classic and gz-sim names), unless disabled.
* A YAML or JSON sensor file (see ``docs/urdf_import.md``) for sensors the
  URDF does not describe, or to override what the Gazebo block says.

Each sensor ends up as a :class:`SensorSpec` holding its pose relative to the
URDF link it is mounted on. Cameras and lidars use the body convention
(x forward, z up); a camera mounted on an ``*_optical_frame`` link must be
declared ``frame: optical`` so it is rotated back to the body convention.
"""

import math
import os
import re

from .ouster import read_ouster_metadata
from .transforms import OPTICAL_FROM_BODY, Transform
from .urdf_model import UrdfError

# ImageType values from AirLib's ImageCaptureBase::ImageType.
IMAGE_TYPES = {
    'Scene': 0, 'DepthPlanar': 1, 'DepthPerspective': 2, 'DepthVis': 3,
    'DisparityNormalized': 4, 'Segmentation': 5, 'SurfaceNormals': 6,
    'Infrared': 7, 'OpticalFlow': 8, 'OpticalFlowVis': 9, 'Annotation': 10,
    'ThermalIR': 11, 'NightVision': 12,
}

# SensorType values from AirLib's SensorBase::SensorType.
SENSOR_TYPES = {
    'barometer': 1, 'imu': 2, 'gps': 3, 'magnetometer': 4, 'distance': 5,
    'lidar': 6, 'gpulidar': 8,
}

KINDS = ('camera', 'lidar', 'gpulidar', 'imu', 'gps', 'barometer',
         'magnetometer', 'distance')
# Sensors HERCULES models at the vehicle origin; a link is optional.
POSELESS = ('imu', 'gps', 'barometer', 'magnetometer')

_GAZEBO_CAMERA_TYPES = {
    'camera': ['Scene'],
    'wideanglecamera': ['Scene'],
    'depth': ['Scene', 'DepthPlanar'],
    'depth_camera': ['DepthPlanar'],
    'rgbd_camera': ['Scene', 'DepthPlanar'],
    'thermal_camera': ['ThermalIR'],
    'thermal': ['ThermalIR'],
    'segmentation_camera': ['Segmentation'],
    'segmentation': ['Segmentation'],
}
_GAZEBO_LIDAR_TYPES = {'ray': 'lidar', 'lidar': 'lidar',
                       'gpu_ray': 'gpulidar', 'gpu_lidar': 'gpulidar'}
_GAZEBO_SIMPLE_TYPES = {'imu': 'imu', 'gps': 'gps', 'navsat': 'gps',
                        'magnetometer': 'magnetometer', 'altimeter': 'barometer',
                        'air_pressure': 'barometer'}


class SensorSpec:

    def __init__(self, kind, name, link=None, offset=None, params=None, source='config'):
        if kind not in KINDS:
            raise UrdfError('unknown sensor kind "%s" (expected one of %s)'
                            % (kind, ', '.join(KINDS)))
        self.kind = kind
        self.name = name
        self.link = link
        self.offset = offset or Transform()
        self.params = dict(params or {})
        self.source = source

    def __repr__(self):
        return 'SensorSpec(%s %r on %r)' % (self.kind, self.name, self.link)


def sanitize_name(name):
    """Make a HERCULES sensor name that is also a valid ROS name and TF frame."""
    clean = re.sub(r'[^A-Za-z0-9_]', '_', name or '').strip('_')
    if not clean:
        raise UrdfError('sensor name "%s" has no usable characters' % name)
    if clean[0].isdigit():
        clean = 's_' + clean
    return clean


def _pose_element(element):
    text = element.findtext('pose') if element is not None else None
    if not text:
        return Transform()
    values = [float(v) for v in text.split()]
    if len(values) != 6:
        raise UrdfError('sensor <pose> must have 6 numbers, got "%s"' % text)
    return Transform.from_xyz_rpy(values[:3], values[3:])


def _num(element, path, default=None, cast=float):
    text = element.findtext(path) if element is not None else None
    if text is None or not text.strip():
        return default
    return cast(text.strip())


def _gazebo_camera(sensor, camera, name, image_types, warnings):
    if camera is None:
        warnings.append('camera sensor "%s" has no <camera> block; using '
                        'Gazebo defaults' % name)
    params = {
        'width': _num(camera, 'image/width', 320, int),
        'height': _num(camera, 'image/height', 240, int),
        'fov_deg': math.degrees(_num(camera, 'horizontal_fov', 1.047)),
        'image_types': list(image_types),
    }
    rate = _num(sensor.element, 'update_rate')
    if rate:
        params['update_rate'] = rate
    return params


def _scan(ray, axis):
    element = ray.find('scan/' + axis) if ray is not None else None
    if element is None:
        return None
    samples = _num(element, 'samples', 1, int)
    resolution = _num(element, 'resolution', 1.0)
    return {
        'samples': max(1, int(round(samples * resolution))),
        'min': _num(element, 'min_angle', 0.0),
        'max': _num(element, 'max_angle', 0.0),
    }


def _whole_rate(element, name, warnings):
    # HERCULES lidars spin at a whole number of rotations per second.
    rate = _num(element, 'update_rate', 10.0)
    whole = max(1, int(round(rate)))
    if abs(whole - rate) > 1e-6:
        warnings.append('lidar "%s": update_rate %g Hz rounded to %d rotations '
                        'per second' % (name, rate, whole))
    return whole


def from_gazebo(model, warnings):
    specs = []
    for sensor in model.gazebo_sensors:
        name = sanitize_name(sensor.name or '%s_%s' % (sensor.link, sensor.type))
        offset = _pose_element(sensor.element)
        kind_type = sensor.type
        if kind_type in _GAZEBO_CAMERA_TYPES:
            camera = sensor.element.find('camera')
            specs.append(SensorSpec(
                'camera', name, sensor.link, offset,
                _gazebo_camera(sensor, camera, name, _GAZEBO_CAMERA_TYPES[kind_type],
                               warnings), 'gazebo'))
            if kind_type == 'wideanglecamera':
                warnings.append('camera "%s": wide-angle lens model is not '
                                'simulated; imported as a pinhole camera' % name)
        elif kind_type == 'multicamera':
            for camera in sensor.element.findall('camera'):
                sub = sanitize_name('%s_%s' % (name, camera.get('name') or len(specs)))
                specs.append(SensorSpec(
                    'camera', sub, sensor.link, offset * _pose_element(camera),
                    _gazebo_camera(sensor, camera, sub, ['Scene'], warnings), 'gazebo'))
        elif kind_type in _GAZEBO_LIDAR_TYPES:
            ray = sensor.element.find('ray')
            if ray is None:
                ray = sensor.element.find('lidar')
            horizontal = _scan(ray, 'horizontal') or {'samples': 1, 'min': 0.0, 'max': 0.0}
            vertical = _scan(ray, 'vertical') or {'samples': 1, 'min': 0.0, 'max': 0.0}
            range_min = _num(ray, 'range/min', 0.0)
            range_max = _num(ray, 'range/max', 10.0)
            if horizontal['samples'] <= 1 and vertical['samples'] <= 1:
                specs.append(SensorSpec('distance', name, sensor.link, offset, {
                    'min': range_min, 'max': range_max}, 'gazebo'))
                continue
            specs.append(SensorSpec(_GAZEBO_LIDAR_TYPES[kind_type], name, sensor.link, offset, {
                'channels': vertical['samples'],
                'range': range_max,
                'rotations_per_second': _whole_rate(sensor.element, name, warnings),
                'measurements_per_cycle': horizontal['samples'],
                'horizontal_fov_deg': [math.degrees(horizontal['min']),
                                       math.degrees(horizontal['max'])],
                'vertical_fov_deg': [math.degrees(vertical['min']),
                                     math.degrees(vertical['max'])],
            }, 'gazebo'))
        elif kind_type == 'sonar':
            sonar = sensor.element.find('sonar')
            specs.append(SensorSpec('distance', name, sensor.link, offset, {
                'min': _num(sonar, 'min', 0.0), 'max': _num(sonar, 'max', 5.0)}, 'gazebo'))
        elif kind_type in _GAZEBO_SIMPLE_TYPES:
            specs.append(SensorSpec(_GAZEBO_SIMPLE_TYPES[kind_type], name, sensor.link,
                                    offset, {}, 'gazebo'))
        else:
            warnings.append('Gazebo sensor "%s" of type "%s" has no HERCULES '
                            'equivalent and was skipped' % (sensor.name, kind_type))
    return specs


_CONFIG_SECTIONS = {
    'cameras': 'camera', 'lidars': 'lidar', 'gpulidars': 'gpulidar', 'imus': 'imu',
    'gps': 'gps', 'barometers': 'barometer', 'magnetometers': 'magnetometer',
    'distances': 'distance',
}
_CONFIG_KEYS = {
    'camera': {'width', 'height', 'fov_deg', 'image_types', 'frame'},
    'lidar': {'channels', 'range', 'rotations_per_second', 'measurements_per_cycle',
              'horizontal_fov_deg', 'vertical_fov_deg', 'draw_debug_points',
              'generate_noise', 'ouster_metadata'},
    'distance': {'min', 'max'},
}
_CONFIG_KEYS['gpulidar'] = _CONFIG_KEYS['lidar']
_COMMON_KEYS = {'name', 'link', 'xyz', 'rpy', 'enabled'}


def from_config(config, model, base_dir='.'):
    """
    Parse the sensor config mapping into SensorSpecs.

    Returns (specs, disabled_names): a sensor listed with ``enabled: false``
    removes a same-named sensor found in the URDF. ``ouster_metadata`` paths
    are relative to ``base_dir``.
    """
    if not isinstance(config, dict):
        raise UrdfError('sensor config must be a mapping of sections')
    unknown = sorted(set(config) - set(_CONFIG_SECTIONS))
    if unknown:
        raise UrdfError('unknown sensor config section(s): %s (expected %s)'
                        % (', '.join(unknown), ', '.join(sorted(_CONFIG_SECTIONS))))
    specs, disabled = [], set()
    for section, kind in _CONFIG_SECTIONS.items():
        entries = config.get(section) or []
        if isinstance(entries, dict):
            entries = [dict(value or {}, name=key) for key, value in entries.items()]
        for entry in entries:
            if not isinstance(entry, dict) or not entry.get('name'):
                raise UrdfError('every entry in "%s" needs a name' % section)
            allowed = _COMMON_KEYS | _CONFIG_KEYS.get(kind, set())
            extra = sorted(set(entry) - allowed)
            if extra:
                raise UrdfError('%s "%s": unknown key(s) %s (allowed: %s)' % (
                    kind, entry['name'], ', '.join(extra), ', '.join(sorted(allowed))))
            name = sanitize_name(entry['name'])
            if entry.get('enabled', True) is False:
                disabled.add(name)
                continue
            link = entry.get('link')
            if link is None and kind not in POSELESS:
                raise UrdfError('%s "%s" needs the URDF link it is mounted on'
                                % (kind, name))
            if link is not None and link not in model.links:
                raise UrdfError('%s "%s" is mounted on unknown link "%s"'
                                % (kind, name, link))
            offset = Transform.from_xyz_rpy(
                [float(v) for v in entry.get('xyz', (0, 0, 0))],
                [float(v) for v in entry.get('rpy', (0, 0, 0))])
            params = {k: v for k, v in entry.items() if k not in _COMMON_KEYS}
            if 'ouster_metadata' in params:
                metadata = os.path.expanduser(params.pop('ouster_metadata'))
                beams = read_ouster_metadata(os.path.join(base_dir, metadata))
                for key in ('channels', 'vertical_fov_deg'):
                    if key in params:
                        raise UrdfError('%s "%s": %s comes from ouster_metadata'
                                        % (kind, name, key))
                params = dict({k: v for k, v in beams.items() if k != 'product'}, **params)
            if kind == 'camera':
                frame = params.pop('frame', 'body')
                if frame not in ('body', 'optical'):
                    raise UrdfError('camera "%s": frame must be "body" or '
                                    '"optical", got "%s"' % (name, frame))
                if frame == 'optical':
                    offset = offset * OPTICAL_FROM_BODY.inverse()
            specs.append(SensorSpec(kind, name, link, offset, params, 'config'))
    return specs, disabled


def merge(gazebo_specs, config_specs, disabled):
    """Config entries replace same-named Gazebo ones; names must be unique."""
    by_name = {}
    for spec in gazebo_specs:
        if spec.name in by_name:
            raise UrdfError('two Gazebo sensors are both named "%s"; rename one '
                            'or override them in the sensor config' % spec.name)
        by_name[spec.name] = spec
    config_names = set()
    for spec in config_specs:
        if spec.name in config_names:
            raise UrdfError('sensor config names "%s" twice' % spec.name)
        config_names.add(spec.name)
        previous = by_name.get(spec.name)
        if previous is not None and previous.kind == spec.kind:
            # Keep Gazebo values the config does not override.
            spec.params = dict(previous.params, **spec.params)
        by_name[spec.name] = spec
    for name in disabled:
        by_name.pop(name, None)
    return list(by_name.values())


def _capture_settings(spec, warnings):
    width = int(spec.params.get('width', 640))
    height = int(spec.params.get('height', 480))
    fov = float(spec.params.get('fov_deg', 90.0))
    if width <= 0 or height <= 0 or not 0.0 < fov < 180.0:
        raise UrdfError('camera "%s": need positive size and 0 < fov_deg < 180'
                        % spec.name)
    captures = []
    for image_type in spec.params.get('image_types', ['Scene']):
        if image_type not in IMAGE_TYPES:
            raise UrdfError('camera "%s": unknown image type "%s" (expected %s)' % (
                spec.name, image_type, ', '.join(IMAGE_TYPES)))
        captures.append({'ImageType': IMAGE_TYPES[image_type], 'Width': width,
                         'Height': height, 'FOV_Degrees': round(fov, 4)})
    if not captures:
        warnings.append('camera "%s" has no image types and will publish nothing'
                        % spec.name)
    return captures


def lidar_settings(spec):
    """
    Build the settings keys of a (GPU) lidar in the sim's angle convention.

    The sim measures azimuth clockwise from x when seen from above (FRD yaw),
    ROS counter-clockwise, so a ROS [min, max] window becomes [-max, -min].
    """
    p = spec.params
    channels = int(p.get('channels', 16))
    measurements = int(p.get('measurements_per_cycle', 1024))
    rps = float(p.get('rotations_per_second', 10))
    rng = float(p.get('range', 100.0))
    if channels < 1 or measurements < 2 or rps <= 0 or rng <= 0:
        raise UrdfError('lidar "%s": channels >= 1, measurements_per_cycle >= 2, '
                        'and positive rotations_per_second and range are required'
                        % spec.name)
    h_min, h_max = (float(v) for v in p.get('horizontal_fov_deg', (-180.0, 180.0)))
    vertical_angles = [float(v) for v in p.get('vertical_angles', ())]
    if vertical_angles:
        channels = len(vertical_angles)
        v_min, v_max = min(vertical_angles), max(vertical_angles)
    else:
        v_min, v_max = (float(v) for v in p.get('vertical_fov_deg', (-15.0, 15.0)))
    if h_min >= h_max or v_min > v_max:
        raise UrdfError('lidar "%s": FOV ranges must be [min, max]' % spec.name)
    settings = {
        'SensorType': SENSOR_TYPES[spec.kind],
        'Enabled': True,
        'NumberOfChannels': channels,
        'Range': rng,
        'MeasurementsPerCycle': measurements,
        'VerticalFOVUpper': round(v_max, 6),
        'VerticalFOVLower': round(v_min, 6),
    }
    # Both lidar models read this key as an integer.
    if abs(rps - round(rps)) > 1e-6:
        raise UrdfError('lidar "%s": rotations_per_second must be a whole '
                        'number, got %s' % (spec.name, rps))
    settings['RotationsPerSecond'] = int(round(rps))
    if vertical_angles and spec.kind == 'lidar':
        # a calibrated beam table (both are in the sim's convention already:
        # elevation positive up, azimuth offset clockwise)
        settings['VerticalAngles'] = [round(v, 6) for v in vertical_angles]
        offsets = [float(v) for v in p.get('azimuth_offsets', ())]
        if offsets:
            if len(offsets) != channels:
                raise UrdfError('lidar "%s": %d azimuth offsets for %d channels'
                                % (spec.name, len(offsets), channels))
            settings['AzimuthOffsets'] = [round(v, 6) for v in offsets]
    if h_max - h_min < 360.0 - 1e-3:
        settings['HorizontalFOVStart'] = round(-h_max, 6)
        settings['HorizontalFOVEnd'] = round(-h_min, 6)
    for key, setting in (('draw_debug_points', 'DrawDebugPoints'),
                         ('generate_noise', 'GenerateNoise')):
        if key in p:
            settings[setting] = bool(p[key])
    return settings


def settings_entry(spec, pose_settings, warnings):
    """Build the settings.json block for one sensor (cameras go under Cameras)."""
    if spec.kind == 'camera':
        entry = dict(pose_settings)
        entry['CaptureSettings'] = _capture_settings(spec, warnings)
        return entry
    entry = {'SensorType': SENSOR_TYPES[spec.kind], 'Enabled': True}
    if spec.kind in ('lidar', 'gpulidar'):
        if spec.kind == 'gpulidar' and spec.params.get('vertical_angles'):
            warnings.append('GPU lidar "%s": the GPU lidar spaces its channels evenly, so the '
                            'beam table is approximated over the same vertical FOV; use a '
                            'CPU lidar (lidars:) for calibrated beam angles' % spec.name)
        entry = lidar_settings(spec)
        entry.update(pose_settings)
    elif spec.kind == 'distance':
        entry.update(pose_settings)
        entry['MinDistance'] = float(spec.params.get('min', 0.2))
        entry['MaxDistance'] = float(spec.params.get('max', 40.0))
        if entry['MinDistance'] >= entry['MaxDistance']:
            raise UrdfError('distance sensor "%s": min must be below max' % spec.name)
    return entry
