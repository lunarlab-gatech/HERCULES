"""
A small, dependency-free URDF reader.

It reads only what the importer needs: the kinematic tree, visual
geometry and materials, and the ``<gazebo reference=...>`` sensor blocks
that robot descriptions commonly carry. Collision and inertial elements are
ignored because HERCULES drives the robot with its own vehicle dynamics.
"""

import math
import xml.etree.ElementTree as ET

from .transforms import quat_from_axis_angle, Transform


class UrdfError(ValueError):
    """The URDF is malformed or uses something the importer cannot map."""


class Geometry:
    """One of ``box``, ``cylinder``, ``sphere`` or ``mesh``."""

    def __init__(self, kind, size=None, radius=None, length=None,
                 filename=None, scale=(1.0, 1.0, 1.0)):
        self.kind = kind
        self.size = size
        self.radius = radius
        self.length = length
        self.filename = filename
        self.scale = scale


class Visual:

    def __init__(self, name, origin, geometry, rgba=None, material_name=None):
        self.name = name
        self.origin = origin
        self.geometry = geometry
        self.rgba = rgba
        self.material_name = material_name


class Link:

    def __init__(self, name):
        self.name = name
        self.visuals = []


class Joint:

    MOVABLE = ('revolute', 'continuous', 'prismatic')
    TYPES = MOVABLE + ('fixed', 'floating', 'planar')

    def __init__(self, name, joint_type, parent, child, origin,
                 axis=(1.0, 0.0, 0.0), lower=None, upper=None, mimic=None):
        self.name = name
        self.type = joint_type
        self.parent = parent
        self.child = child
        self.origin = origin
        self.axis = axis
        self.lower = lower
        self.upper = upper
        # (joint, multiplier, offset)
        self.mimic = mimic

    @property
    def movable(self):
        return self.type in self.MOVABLE

    def default_position(self):
        """Mirror joint_state_publisher: zero, or mid-range if zero is out."""
        if self.lower is not None and self.upper is not None and \
                self.type != 'continuous' and not self.lower <= 0.0 <= self.upper:
            return (self.lower + self.upper) / 2.0
        return 0.0

    def motion(self, position):
        if self.type in ('revolute', 'continuous'):
            return Transform(quat_from_axis_angle(self.axis, position))
        if self.type == 'prismatic':
            return Transform(translation=tuple(a * position for a in self.axis))
        return Transform()


class GazeboSensor:
    """A ``<sensor>`` element found under ``<gazebo reference="link">``."""

    def __init__(self, link, element):
        self.link = link
        self.element = element
        self.name = element.get('name') or ''
        self.type = (element.get('type') or '').strip()


class UrdfModel:

    def __init__(self, name, links, joints, gazebo_sensors, materials):
        self.name = name
        self.links = links
        self.joints = joints
        self.gazebo_sensors = gazebo_sensors
        self.materials = materials
        self._child_joint = {j.child: j for j in joints.values()}
        self.root = self._find_root()

    def _find_root(self):
        roots = [name for name in self.links if name not in self._child_joint]
        if len(roots) != 1:
            raise UrdfError('URDF must have exactly one root link, found %s'
                            % (sorted(roots) or 'none (cycle)'))
        # Every link must reach the root without revisiting a link.
        for name in self.links:
            seen = set()
            while name in self._child_joint:
                if name in seen:
                    raise UrdfError('kinematic loop through link "%s"' % name)
                seen.add(name)
                name = self._child_joint[name].parent
        return roots[0]

    def parent_joint(self, link):
        return self._child_joint.get(link)

    def joint_positions(self, overrides=None):
        """Resolve every movable joint's position, honoring ``<mimic>``."""
        overrides = dict(overrides or {})
        unknown = sorted(set(overrides) - set(self.joints))
        if unknown:
            raise UrdfError('unknown joint(s) in overrides: %s' % ', '.join(unknown))
        positions = {}

        def resolve(joint, depth=0):
            if joint.name in positions:
                return positions[joint.name]
            if depth > len(self.joints):
                raise UrdfError('mimic loop at joint "%s"' % joint.name)
            if joint.name in overrides:
                value = float(overrides[joint.name])
            elif joint.mimic is not None:
                source, multiplier, offset = joint.mimic
                if source not in self.joints:
                    raise UrdfError('joint "%s" mimics unknown joint "%s"'
                                    % (joint.name, source))
                value = multiplier * resolve(self.joints[source], depth + 1) + offset
            else:
                value = joint.default_position()
            positions[joint.name] = value
            return value

        for joint in self.joints.values():
            if joint.movable:
                resolve(joint)
        return positions

    def link_poses(self, joint_positions=None):
        """Pose of every link relative to the root link."""
        joint_positions = joint_positions or {}
        poses = {self.root: Transform()}

        def pose(link):
            if link not in poses:
                joint = self._child_joint[link]
                local = joint.origin * joint.motion(joint_positions.get(joint.name, 0.0))
                poses[link] = pose(joint.parent) * local
            return poses[link]

        for name in self.links:
            pose(name)
        return poses

    def chain_to_root(self, link):
        """Joints from ``link`` up to the root, nearest first."""
        chain = []
        while link in self._child_joint:
            joint = self._child_joint[link]
            chain.append(joint)
            link = joint.parent
        return chain


def _floats(text, count, what):
    try:
        values = tuple(float(v) for v in (text or '').split())
    except ValueError:
        raise UrdfError('%s: expected %d numbers, got "%s"' % (what, count, text))
    if len(values) != count:
        raise UrdfError('%s: expected %d numbers, got "%s"' % (what, count, text))
    if not all(math.isfinite(v) for v in values):
        raise UrdfError('%s: non-finite value in "%s"' % (what, text))
    return values


def parse_origin(element, what='origin'):
    if element is None:
        return Transform()
    xyz = _floats(element.get('xyz', '0 0 0'), 3, what + ' xyz')
    rpy = _floats(element.get('rpy', '0 0 0'), 3, what + ' rpy')
    return Transform.from_xyz_rpy(xyz, rpy)


def _parse_material(element, materials, what):
    """Return (name, rgba or None) for a <material> element."""
    if element is None:
        return None, None
    name = element.get('name')
    color = element.find('color')
    if color is not None:
        rgba = _floats(color.get('rgba'), 4, what + ' material rgba')
        return name, rgba
    if name and name in materials:
        return name, materials[name]
    return name, None


def _parse_geometry(element, what):
    if element is None or len(element) == 0:
        raise UrdfError('%s has no geometry' % what)
    shape = element[0]
    if shape.tag == 'box':
        size = _floats(shape.get('size'), 3, what + ' box size')
        return Geometry('box', size=size)
    if shape.tag == 'cylinder':
        return Geometry('cylinder',
                        radius=_floats(shape.get('radius'), 1, what + ' radius')[0],
                        length=_floats(shape.get('length'), 1, what + ' length')[0])
    if shape.tag == 'sphere':
        return Geometry('sphere',
                        radius=_floats(shape.get('radius'), 1, what + ' radius')[0])
    if shape.tag == 'mesh':
        filename = shape.get('filename')
        if not filename:
            raise UrdfError('%s mesh has no filename' % what)
        scale = shape.get('scale')
        return Geometry('mesh', filename=filename,
                        scale=_floats(scale, 3, what + ' mesh scale') if scale
                        else (1.0, 1.0, 1.0))
    raise UrdfError('%s uses unsupported geometry <%s>' % (what, shape.tag))


def parse_urdf_string(text):
    try:
        root = ET.fromstring(text)
    except ET.ParseError as exc:
        raise UrdfError('URDF is not well-formed XML: %s' % exc)
    if root.tag != 'robot':
        raise UrdfError('root element is <%s>, expected <robot>' % root.tag)
    if any('/xacro}' in el.tag for el in root.iter()) or any(
            '${' in value for el in root.iter() for value in el.attrib.values()):
        raise UrdfError('this looks like an unexpanded xacro file; pass it '
                        'through xacro first (the importer does this for '
                        '*.xacro inputs when the xacro module is installed)')

    materials = {}
    for element in root.findall('material'):
        name, rgba = _parse_material(element, materials, 'material')
        if name and rgba is not None:
            materials[name] = rgba

    links = {}
    for element in root.findall('link'):
        name = element.get('name')
        if not name:
            raise UrdfError('<link> without a name')
        if name in links:
            raise UrdfError('duplicate link "%s"' % name)
        link = Link(name)
        for index, visual in enumerate(element.findall('visual')):
            what = 'link "%s" visual %d' % (name, index)
            mat_name, rgba = _parse_material(visual.find('material'), materials, what)
            link.visuals.append(Visual(
                visual.get('name') or '%s_visual_%d' % (name, index),
                parse_origin(visual.find('origin'), what + ' origin'),
                _parse_geometry(visual.find('geometry'), what),
                rgba, mat_name))
        links[name] = link
    if not links:
        raise UrdfError('URDF has no links')

    joints = {}
    for element in root.findall('joint'):
        name = element.get('name')
        joint_type = element.get('type')
        what = 'joint "%s"' % name
        if not name or joint_type not in Joint.TYPES:
            raise UrdfError('%s has invalid type "%s"' % (what, joint_type))
        if name in joints:
            raise UrdfError('duplicate joint "%s"' % name)
        parent = element.find('parent')
        child = element.find('child')
        if parent is None or child is None:
            raise UrdfError('%s needs <parent> and <child>' % what)
        parent, child = parent.get('link'), child.get('link')
        for link in (parent, child):
            if link not in links:
                raise UrdfError('%s references unknown link "%s"' % (what, link))
        axis = (1.0, 0.0, 0.0)
        if element.find('axis') is not None:
            axis = _floats(element.find('axis').get('xyz'), 3, what + ' axis')
            if not any(axis):
                raise UrdfError('%s has a zero axis' % what)
        lower = upper = None
        limit = element.find('limit')
        if limit is not None:
            if limit.get('lower') is not None:
                lower = float(limit.get('lower'))
            if limit.get('upper') is not None:
                upper = float(limit.get('upper'))
        if joint_type == 'revolute' and (lower is None or upper is None):
            # urdfdom treats missing limits as 0; keep that behavior.
            lower = 0.0 if lower is None else lower
            upper = 0.0 if upper is None else upper
        mimic = None
        mimic_el = element.find('mimic')
        if mimic_el is not None:
            mimic = (mimic_el.get('joint'),
                     float(mimic_el.get('multiplier', 1.0)),
                     float(mimic_el.get('offset', 0.0)))
        joints[name] = Joint(name, joint_type, parent, child,
                             parse_origin(element.find('origin'), what + ' origin'),
                             axis, lower, upper, mimic)
    children = [j.child for j in joints.values()]
    duplicates = sorted({c for c in children if children.count(c) > 1})
    if duplicates:
        raise UrdfError('link(s) with more than one parent joint: %s'
                        % ', '.join(duplicates))

    gazebo_sensors = []
    for gazebo in root.findall('gazebo'):
        reference = gazebo.get('reference')
        for sensor in gazebo.findall('sensor'):
            if reference is None:
                continue
            if reference not in links:
                raise UrdfError('<gazebo reference="%s"> names an unknown link'
                                % reference)
            gazebo_sensors.append(GazeboSensor(reference, sensor))

    return UrdfModel(root.get('name') or 'robot', links, joints,
                     gazebo_sensors, materials)


def parse_urdf_file(path):
    with open(path, 'r', encoding='utf-8') as stream:
        return parse_urdf_string(stream.read())
