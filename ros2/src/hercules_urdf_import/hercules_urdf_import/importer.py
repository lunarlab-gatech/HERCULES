"""
Turn a URDF into a HERCULES vehicle.

``import_robot`` produces, in one output directory:

* ``<vehicle>.settings.json`` - a settings file (or the merge of the vehicle
  into an existing one) with the vehicle type, pawn, cameras and sensors at
  the poses the URDF gives them;
* ``<vehicle>.visuals.json`` and ``meshes/*.stl`` - the visual manifest the
  Unreal plugin attaches to the vehicle's pawn;
* ``<vehicle>.urdf`` - the URDF robot_state_publisher should publish: rooted
  at the HERCULES body frame, with the camera/lidar frames the ROS wrapper
  stamps its messages with, and movable joints frozen at the pose the
  simulator renders.
"""

import hashlib
import json
import math
import os
import pathlib
import re
import xml.etree.ElementTree as ET

from . import meshes
from . import sensors as sensor_mod
from .resolve import PackageResolver
from .transforms import flu_to_frd_quat, OPTICAL_FROM_BODY, settings_pose, Transform
from .urdf_model import parse_urdf_string, UrdfError

MANIFEST_FORMAT = 'hercules-urdf-visuals'
MANIFEST_VERSION = 1
DEFAULT_COLOR = (0.7, 0.7, 0.7, 1.0)
TRIANGLE_BUDGET = 500000

# The UGV pawn the Hero team configs use for skid-steer robots.
UGV_PAWN = {'UGVPawn': {'PawnBP': "Class'/AirSim/VehicleAdv/SUV/UGVPawn.UGVPawn_C'"}}

# drive -> {sim mode: (SimMode, VehicleType, PawnPath or None)}
DRIVES = {
    'skid': {'hero': ('Hero', 'PhysXCar', 'UGVPawn'),
             'native': ('SkidVehicle', 'CPHusky', None)},
    'diff': {'hero': ('Hero', 'PhysXCar', 'UGVPawn'),
             'native': ('SkidVehicle', 'Pioneer', None)},
    'ackermann': {'hero': ('Hero', 'PhysXCar', None),
                  'native': ('Car', 'PhysXCar', None)},
    'multirotor': {'hero': ('Hero', 'SimpleFlight', None),
                   'native': ('Multirotor', 'SimpleFlight', None)},
    'static': {'native': ('ComputerVision', 'ComputerVision', None)},
}

_VEHICLE_NAME = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')


class ImportOptions:

    def __init__(self, urdf_path, vehicle_name, drive, output_dir,
                 sim_mode='hero', base_link=None, joint_positions=None,
                 sensor_config=None, gazebo_sensors=True, position=(0.0, 0.0, 0.0),
                 yaw_deg=0.0, hide_base_mesh=True, freeze_joints=True,
                 relative_paths=False, package_paths=None, odom_frame='odom_local',
                 merge_settings=None, vehicle_type=None, pawn_path=None,
                 rewrite_mesh_uris=True, xacro_args=None, two_sided=False):
        self.urdf_path = urdf_path
        self.vehicle_name = vehicle_name
        self.drive = drive
        self.output_dir = output_dir
        self.sim_mode = sim_mode
        self.base_link = base_link
        self.joint_positions = dict(joint_positions or {})
        self.sensor_config = sensor_config
        self.gazebo_sensors = gazebo_sensors
        self.position = tuple(float(v) for v in position)
        self.yaw_deg = float(yaw_deg)
        self.hide_base_mesh = hide_base_mesh
        self.freeze_joints = freeze_joints
        self.relative_paths = relative_paths
        self.package_paths = dict(package_paths or {})
        self.odom_frame = odom_frame
        self.merge_settings = merge_settings
        self.vehicle_type = vehicle_type
        self.pawn_path = pawn_path
        self.rewrite_mesh_uris = rewrite_mesh_uris
        self.xacro_args = dict(xacro_args or {})
        self.two_sided = two_sided


class ImportResult:

    def __init__(self):
        self.settings = None
        self.vehicle = None
        self.manifest = None
        self.urdf = None
        self.sensors = []
        self.warnings = []
        self.files = {}
        self.triangles = 0


def default_base_link(model, drive):
    """
    Pick the link that coincides with the pawn origin.

    Wheeled pawns have their origin at ground level, which REP-120 calls
    base_footprint; flying and static pawns are centered on base_link.
    """
    names = ('base_footprint', 'base_link') if drive in ('skid', 'diff', 'ackermann') \
        else ('base_link',)
    return next((name for name in names if name in model.links), model.root)


def load_urdf_text(path, xacro_args=None):
    if path.endswith('.xacro'):
        try:
            import xacro
        except ImportError:
            raise UrdfError('%s is a xacro file but the xacro Python module is '
                            'not installed; run `xacro %s > robot.urdf` first'
                            % (path, path))
        document = xacro.process_file(path, mappings=dict(xacro_args or {}))
        return document.toxml()
    with open(path, 'r', encoding='utf-8') as stream:
        return stream.read()


def _sensor_config(source):
    if source is None or isinstance(source, dict):
        return source
    with open(source, 'r', encoding='utf-8') as stream:
        text = stream.read()
    if source.endswith('.json'):
        return json.loads(text)
    try:
        import yaml
    except ImportError:
        raise UrdfError('reading %s needs PyYAML; install it or use JSON' % source)
    return yaml.safe_load(text) or {}


def _movable_between(model, a, b):
    """Movable joints on the tree path between links a and b."""
    chain_a = {j.name: j for j in model.chain_to_root(a)}
    chain_b = {j.name: j for j in model.chain_to_root(b)}
    path = set(chain_a) ^ set(chain_b)
    return sorted(name for name in path if model.joints[name].movable)


def _quat_dict(q):
    return {'w': q[0], 'x': q[1], 'y': q[2], 'z': q[3]}


def _round(values, digits=9):
    return [0.0 if round(v, digits) == 0 else round(v, digits) for v in values]


def _mesh_file_stem(source, scale, part=0, hash_key=None):
    digest = hashlib.sha1(('%s|%r|%d|%s' % (source, tuple(scale), part, hash_key))
                          .encode()).hexdigest()[:8]
    stem = re.sub(r'[^A-Za-z0-9_]+', '_', os.path.splitext(os.path.basename(source))[0])
    return '%s_%s' % (stem.strip('_') or 'mesh', digest)


def _origin_attrs(transform):
    roll, pitch, yaw = transform.rpy()
    return {'xyz': ' '.join(repr(v) for v in _round(transform.translation)),
            'rpy': ' '.join(repr(v) for v in _round((roll, pitch, yaw)))}


def _add_fixed_joint(robot, name, parent, child, transform):
    joint = ET.SubElement(robot, 'joint', {'name': name, 'type': 'fixed'})
    ET.SubElement(joint, 'parent', {'link': parent})
    ET.SubElement(joint, 'child', {'link': child})
    ET.SubElement(joint, 'origin', _origin_attrs(transform))


def import_robot(options):
    result = ImportResult()
    warnings = result.warnings
    if not _VEHICLE_NAME.match(options.vehicle_name or ''):
        raise UrdfError('vehicle name "%s" must start with a letter and use only '
                        'letters, digits and _ (it becomes ROS topic and TF names)'
                        % options.vehicle_name)
    if options.drive not in DRIVES:
        raise UrdfError('drive must be one of %s' % ', '.join(sorted(DRIVES)))
    modes = DRIVES[options.drive]
    if options.sim_mode not in modes:
        raise UrdfError('a "%s" robot cannot run in the %s sim mode; use --sim-mode %s'
                        % (options.drive, options.sim_mode, ' or '.join(sorted(modes))))
    sim_mode, vehicle_type, pawn_path = modes[options.sim_mode]
    vehicle_type = options.vehicle_type or vehicle_type
    pawn_path = options.pawn_path if options.pawn_path is not None else pawn_path

    urdf_text = load_urdf_text(options.urdf_path, options.xacro_args)
    model = parse_urdf_string(urdf_text)
    urdf_dir = os.path.dirname(os.path.abspath(options.urdf_path))
    resolver = PackageResolver(urdf_dir, options.package_paths)

    base = options.base_link or default_base_link(model, options.drive)
    if base not in model.links:
        raise UrdfError('base link "%s" is not in the URDF' % base)
    positions = model.joint_positions(options.joint_positions)
    root_poses = model.link_poses(positions)
    body_from_root = root_poses[base].inverse()
    body_poses = {name: body_from_root * pose for name, pose in root_poses.items()}

    # ---------------------------------------------------------------- sensors
    gazebo_specs = sensor_mod.from_gazebo(model, warnings) if options.gazebo_sensors else []
    config_specs, disabled = [], set()
    config = _sensor_config(options.sensor_config)
    if config:
        base_dir = os.path.dirname(os.path.abspath(options.sensor_config)) \
            if isinstance(options.sensor_config, str) else os.getcwd()
        config_specs, disabled = sensor_mod.from_config(config, model, base_dir)
    specs = sensor_mod.merge(gazebo_specs, config_specs, disabled)
    if options.drive == 'multirotor' and sim_mode == 'Multirotor' and \
            any(s.kind == 'gpulidar' for s in specs):
        raise UrdfError('the Multirotor sim mode does not support GPU lidars; '
                        'use --sim-mode hero or a CPU lidar')

    cameras, sensor_settings, sensor_frames, framed_sensors = {}, {}, [], []
    for spec in sorted(specs, key=lambda s: (s.kind, s.name)):
        pose = None
        if spec.link is not None:
            pose = body_poses[spec.link] * spec.offset
            moving = _movable_between(model, base, spec.link)
            if moving:
                warnings.append('%s "%s" sits behind movable joint(s) %s; it is '
                                'fixed at the import pose'
                                % (spec.kind, spec.name, ', '.join(moving)))
        if spec.kind in sensor_mod.POSELESS:
            if spec.kind == 'imu' and pose is not None:
                if math.dist(pose.translation, (0.0, 0.0, 0.0)) > 1e-6:
                    warnings.append('imu "%s" is offset from %s; HERCULES measures at '
                                    'the body origin (no lever-arm effects)'
                                    % (spec.name, base))
                if not Transform(pose.rotation).is_close(Transform(), 1e-6):
                    warnings.append('imu "%s" is rotated relative to %s; its data is '
                                    'reported in the body frame, not the IMU link'
                                    % (spec.name, base))
            entry = sensor_mod.settings_entry(spec, {}, warnings)
        else:
            entry = sensor_mod.settings_entry(spec, settings_pose(pose), warnings)
        if spec.kind == 'camera':
            cameras[spec.name] = entry
            framed_sensors.append(spec.name)
            sensor_frames.append((spec.name + '_body', spec.link, spec.offset))
            sensor_frames.append((spec.name + '_optical', spec.name + '_body',
                                  OPTICAL_FROM_BODY))
        else:
            sensor_settings[spec.name] = entry
            if spec.kind in ('lidar', 'gpulidar', 'distance'):
                framed_sensors.append(spec.name)
                sensor_frames.append((spec.name, spec.link, spec.offset))
        result.sensors.append({
            'name': spec.name, 'kind': spec.kind, 'link': spec.link,
            'source': spec.source,
            # the pose written to settings (none for sensors HERCULES models at
            # the vehicle origin, even when the URDF mounts them elsewhere)
            'frd_pose': None if pose is None or spec.kind in sensor_mod.POSELESS else {
                'position': [settings_pose(pose)[k] for k in ('X', 'Y', 'Z')],
                'orientation': _quat_dict(_round(flu_to_frd_quat(pose.rotation)))},
        })

    # ---------------------------------------------------------------- visuals
    out_dir = os.path.abspath(options.output_dir)
    mesh_dir = os.path.join(out_dir, 'meshes')
    os.makedirs(mesh_dir, exist_ok=True)
    written = {}
    visuals = []
    for link_name in sorted(model.links):
        link = model.links[link_name]
        for visual in link.visuals:
            geometry = visual.geometry
            if geometry.kind == 'mesh':
                try:
                    source = resolver.resolve(geometry.filename)
                except UrdfError as exc:
                    warnings.append('link "%s": %s; visual skipped' % (link_name, exc))
                    continue
                key = (source, tuple(geometry.scale))
                if key not in written:
                    try:
                        parts = meshes.read_mesh(source)
                    except (meshes.MeshError, OSError) as exc:
                        warnings.append('link "%s": %s; visual skipped' % (link_name, exc))
                        continue
                    written[key] = [
                        (meshes.scale_triangles(part.triangles, geometry.scale), part.rgba,
                         _mesh_file_stem(source, geometry.scale, index))
                        for index, part in enumerate(parts)]
                parts = written[key]
            else:
                key = (geometry.kind, geometry.size, geometry.radius, geometry.length)
                if key not in written:
                    if geometry.kind == 'box':
                        tris = meshes.box(geometry.size)
                    elif geometry.kind == 'cylinder':
                        tris = meshes.cylinder(geometry.radius, geometry.length)
                    else:
                        tris = meshes.sphere(geometry.radius)
                    written[key] = [(tris, None, _mesh_file_stem(
                        geometry.kind + '.stl', (1, 1, 1), hash_key=repr(key)))]
                parts = written[key]
            pose = body_poses[link_name] * visual.origin
            for index, (tris, part_rgba, stem) in enumerate(parts):
                filename = stem + '.stl'
                path = os.path.join(mesh_dir, filename)
                if path not in result.files.values():
                    meshes.write_binary_stl(path, tris)
                    result.files['mesh:' + filename] = path
                    result.triangles += len(tris)
                rgba = part_rgba or visual.rgba or DEFAULT_COLOR
                visuals.append({
                    'name': visual.name if len(parts) == 1 else '%s_%d' % (visual.name, index),
                    'link': link_name,
                    'mesh': 'meshes/' + filename,
                    'position': _round(pose.translation),
                    'orientation': _quat_dict(_round(pose.rotation)),
                    'scale': [1.0, 1.0, 1.0],
                    'color': _round(rgba, 6),
                })
    if not visuals:
        warnings.append('no visual could be converted; the pawn keeps its own mesh')
    if result.triangles > TRIANGLE_BUDGET:
        warnings.append('%d triangles: consider decimating the meshes, the pawn is '
                        'rebuilt from them at every simulation start' % result.triangles)

    manifest = {
        'format': MANIFEST_FORMAT,
        'version': MANIFEST_VERSION,
        'robot': model.name,
        'vehicle': options.vehicle_name,
        'frame': base,
        'frame_convention': 'FLU',
        'units': 'meters',
        'visuals': visuals,
        'sensors': result.sensors,
    }
    result.manifest = manifest

    # ------------------------------------------------------- augmented URDF
    robot = ET.fromstring(urdf_text)
    existing = {el.get('name') for el in robot.findall('link')}
    body_frame = 'ground_truth/' + options.odom_frame
    for frame, _, _ in sensor_frames + [(body_frame, None, None)]:
        if frame in existing:
            raise UrdfError('the URDF already has a link named "%s", which the '
                            'HERCULES ROS wrapper uses for its own frame' % frame)
    if options.freeze_joints:
        for element in robot.findall('joint'):
            joint = model.joints[element.get('name')]
            if not joint.movable:
                continue
            frozen = joint.origin * joint.motion(positions[joint.name])
            element.set('type', 'fixed')
            for tag in ('axis', 'limit', 'mimic', 'dynamics', 'safety_controller',
                        'calibration', 'origin'):
                for child in element.findall(tag):
                    element.remove(child)
            ET.SubElement(element, 'origin', _origin_attrs(frozen))
    if options.rewrite_mesh_uris:
        for mesh in robot.iter('mesh'):
            try:
                mesh.set('filename', pathlib.Path(resolver.resolve(mesh.get('filename'))).as_uri())
            except UrdfError:
                pass
    ET.SubElement(robot, 'link', {'name': body_frame})
    _add_fixed_joint(robot, 'hercules_body_joint', body_frame, model.root,
                     root_poses[base].inverse())
    for frame, parent, transform in sensor_frames:
        ET.SubElement(robot, 'link', {'name': frame})
        _add_fixed_joint(robot, frame + '_joint', parent, frame, transform)
    ET.indent(robot)
    result.urdf = ('<?xml version="1.0"?>\n<!-- Generated by hercules_urdf_import '
                   'from %s. -->\n' % os.path.basename(options.urdf_path)
                   + ET.tostring(robot, encoding='unicode') + '\n')

    # ------------------------------------------------------------- settings
    def ref(path):
        return os.path.relpath(path, out_dir) if options.relative_paths else path

    manifest_path = os.path.join(out_dir, options.vehicle_name + '.visuals.json')
    urdf_path = os.path.join(out_dir, options.vehicle_name + '.urdf')
    x, y, z = options.position
    vehicle = {'VehicleType': vehicle_type}
    if pawn_path:
        vehicle['PawnPath'] = pawn_path
    vehicle.update({'X': x, 'Y': y, 'Z': z, 'Roll': 0.0, 'Pitch': 0.0, 'Yaw': options.yaw_deg})
    vehicle['Urdf'] = {
        'Visuals': ref(manifest_path),
        'RobotDescription': ref(urdf_path),
        'HideBaseMesh': bool(options.hide_base_mesh),
        'TwoSided': bool(options.two_sided),
        # the ROS wrapper leaves these sensors' TF frames to robot_state_publisher
        'SensorFrames': sorted(framed_sensors),
    }
    if cameras:
        vehicle['Cameras'] = cameras
    if sensor_settings:
        vehicle['Sensors'] = sensor_settings
    result.vehicle = vehicle

    if options.merge_settings:
        with open(options.merge_settings, 'r', encoding='utf-8') as stream:
            settings = json.load(stream)
        existing_mode = settings.get('SimMode', '')
        if existing_mode and existing_mode != sim_mode:
            raise UrdfError('%s uses SimMode "%s" but this robot needs "%s"'
                            % (options.merge_settings, existing_mode, sim_mode))
        if options.vehicle_name in settings.get('Vehicles', {}):
            warnings.append('replaced the existing vehicle "%s" in %s'
                            % (options.vehicle_name, options.merge_settings))
    else:
        settings = {'SettingsVersion': 2.0}
    settings['SimMode'] = sim_mode
    if pawn_path == 'UGVPawn':
        settings.setdefault('PawnPaths', {}).update(
            {k: v for k, v in UGV_PAWN.items() if k not in settings.get('PawnPaths', {})})
    settings.setdefault('Vehicles', {})[options.vehicle_name] = vehicle
    result.settings = settings

    # ---------------------------------------------------------------- write
    settings_path = os.path.join(out_dir, options.vehicle_name + '.settings.json')
    for path, content in ((manifest_path, json.dumps(manifest, indent=2) + '\n'),
                          (urdf_path, result.urdf),
                          (settings_path, json.dumps(settings, indent=2) + '\n')):
        with open(path, 'w', encoding='utf-8') as stream:
            stream.write(content)
    result.files.update({'manifest': manifest_path, 'urdf': urdf_path,
                         'settings': settings_path})
    return result
