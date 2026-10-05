"""
Mesh loading and conversion to binary STL.

The Unreal side of HERCULES loads one format, binary STL, so every URDF
visual is converted here: STL (binary or ASCII), Wavefront OBJ (with MTL
diffuse colors) and Collada (honoring ``<unit>``, ``<up_axis>``, node
transforms and per-material colors) meshes, and box/cylinder/sphere
primitives. Output vertices are in the visual's geometry frame, meters,
with the URDF mesh scale already applied, and triangles wound
counter-clockwise when seen from outside (right-handed).
"""

import math
import os
import struct
import xml.etree.ElementTree as ET


class MeshError(ValueError):

    pass


class MeshPart:
    """Triangles that share one color. ``rgba`` is None when unknown."""

    def __init__(self, triangles, rgba=None):
        self.triangles = triangles
        self.rgba = rgba


# --------------------------------------------------------------------------
# Primitives (URDF semantics: centered on the geometry origin, cylinder
# axis along z).

def _quad(a, b, c, d):
    return [(a, b, c), (a, c, d)]


def box(size):
    sx, sy, sz = (s / 2.0 for s in size)
    v = [(x, y, z) for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)]
    # v index = 4*ix + 2*iy + iz
    faces = []
    faces += _quad(v[0], v[1], v[3], v[2])  # -x
    faces += _quad(v[4], v[6], v[7], v[5])  # +x
    faces += _quad(v[0], v[4], v[5], v[1])  # -y
    faces += _quad(v[2], v[3], v[7], v[6])  # +y
    faces += _quad(v[0], v[2], v[6], v[4])  # -z
    faces += _quad(v[1], v[5], v[7], v[3])  # +z
    return faces


def cylinder(radius, length, segments=32):
    h = length / 2.0
    ring = [(radius * math.cos(2 * math.pi * i / segments),
             radius * math.sin(2 * math.pi * i / segments)) for i in range(segments)]
    faces = []
    top, bottom = (0.0, 0.0, h), (0.0, 0.0, -h)
    for i in range(segments):
        (x0, y0), (x1, y1) = ring[i], ring[(i + 1) % segments]
        faces += _quad((x0, y0, -h), (x1, y1, -h), (x1, y1, h), (x0, y0, h))
        faces.append((top, (x0, y0, h), (x1, y1, h)))
        faces.append((bottom, (x1, y1, -h), (x0, y0, -h)))
    return faces


def sphere(radius, stacks=16, slices=32):
    def point(i, j):
        theta = math.pi * i / stacks
        phi = 2 * math.pi * j / slices
        return (radius * math.sin(theta) * math.cos(phi),
                radius * math.sin(theta) * math.sin(phi),
                radius * math.cos(theta))

    faces = []
    for i in range(stacks):
        for j in range(slices):
            a, b = point(i, j), point(i + 1, j)
            c, d = point(i + 1, j + 1), point(i, j + 1)
            if i != 0:
                faces.append((a, b, d))
            if i != stacks - 1:
                faces.append((b, c, d))
    return faces


# --------------------------------------------------------------------------
# STL

def read_stl(path):
    with open(path, 'rb') as stream:
        data = stream.read()
    if len(data) >= 84:
        count = struct.unpack_from('<I', data, 80)[0]
        if len(data) == 84 + 50 * count:
            triangles = []
            for i in range(count):
                values = struct.unpack_from('<12f', data, 84 + 50 * i)
                triangles.append((values[3:6], values[6:9], values[9:12]))
            return [MeshPart(triangles)]
    text = data.decode('ascii', errors='replace')
    if not text.lstrip().startswith('solid'):
        raise MeshError('%s is neither binary nor ASCII STL' % path)
    vertices = []
    for line in text.splitlines():
        fields = line.split()
        if fields and fields[0] == 'vertex':
            if len(fields) != 4:
                raise MeshError('%s: bad vertex line "%s"' % (path, line.strip()))
            vertices.append(tuple(float(f) for f in fields[1:4]))
    if len(vertices) % 3:
        raise MeshError('%s: vertex count is not a multiple of 3' % path)
    return [MeshPart([tuple(vertices[i:i + 3]) for i in range(0, len(vertices), 3)])]


def _normal(a, b, c):
    ux, uy, uz = (b[i] - a[i] for i in range(3))
    vx, vy, vz = (c[i] - a[i] for i in range(3))
    n = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
    length = math.sqrt(sum(x * x for x in n))
    return tuple(x / length for x in n) if length > 0 else (0.0, 0.0, 0.0)


def write_binary_stl(path, triangles, header=b'hercules_urdf_import'):
    with open(path, 'wb') as stream:
        stream.write(header[:80].ljust(80, b'\0'))
        stream.write(struct.pack('<I', len(triangles)))
        for a, b, c in triangles:
            stream.write(struct.pack('<12fH', *_normal(a, b, c), *a, *b, *c, 0))


# --------------------------------------------------------------------------
# Wavefront OBJ

def _read_mtl(path):
    colors = {}
    current = None
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as stream:
            for line in stream:
                fields = line.split()
                if not fields:
                    continue
                if fields[0] == 'newmtl' and len(fields) > 1:
                    current = fields[1]
                    colors[current] = [0.8, 0.8, 0.8, 1.0]
                elif current and fields[0] == 'Kd' and len(fields) >= 4:
                    colors[current][:3] = [float(f) for f in fields[1:4]]
                elif current and fields[0] == 'd' and len(fields) >= 2:
                    colors[current][3] = float(fields[1])
                elif current and fields[0] == 'Tr' and len(fields) >= 2:
                    colors[current][3] = 1.0 - float(fields[1])
    except OSError:
        return {}
    return {k: tuple(v) for k, v in colors.items()}


def read_obj(path):
    vertices = []
    groups = {}
    materials = {}
    current = None
    base = os.path.dirname(path)
    with open(path, 'r', encoding='utf-8', errors='replace') as stream:
        for number, line in enumerate(stream, 1):
            fields = line.split()
            if not fields:
                continue
            if fields[0] == 'v':
                vertices.append(tuple(float(f) for f in fields[1:4]))
            elif fields[0] == 'mtllib':
                for name in fields[1:]:
                    materials.update(_read_mtl(os.path.join(base, name)))
            elif fields[0] == 'usemtl':
                current = fields[1] if len(fields) > 1 else None
            elif fields[0] == 'f':
                indices = []
                for field in fields[1:]:
                    index = int(field.split('/')[0])
                    index = index - 1 if index > 0 else len(vertices) + index
                    if not 0 <= index < len(vertices):
                        raise MeshError('%s:%d: face index out of range' % (path, number))
                    indices.append(vertices[index])
                faces = groups.setdefault(current, [])
                for i in range(1, len(indices) - 1):
                    faces.append((indices[0], indices[i], indices[i + 1]))
    return [MeshPart(faces, materials.get(name)) for name, faces in groups.items() if faces]


# --------------------------------------------------------------------------
# Collada

def _local(tag):
    return tag.rsplit('}', 1)[-1]


def _strip_namespaces(root):
    for element in root.iter():
        element.tag = _local(element.tag)
    return root


def _mat_identity():
    return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def _mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _mat_apply(m, p):
    return tuple(m[i][0] * p[0] + m[i][1] * p[1] + m[i][2] * p[2] + m[i][3] for i in range(3))


def _det3(m):
    return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))


def _node_matrix(node):
    m = _mat_identity()
    for child in node:
        values = [float(v) for v in (child.text or '').split()]
        if child.tag == 'matrix' and len(values) == 16:
            step = [values[i * 4:(i + 1) * 4] for i in range(4)]
        elif child.tag == 'translate' and len(values) == 3:
            step = _mat_identity()
            for i in range(3):
                step[i][3] = values[i]
        elif child.tag == 'scale' and len(values) == 3:
            step = _mat_identity()
            for i in range(3):
                step[i][i] = values[i]
        elif child.tag == 'rotate' and len(values) == 4:
            x, y, z, angle = values
            n = math.sqrt(x * x + y * y + z * z) or 1.0
            x, y, z = x / n, y / n, z / n
            c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
            t = 1 - c
            step = [[t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0],
                    [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0],
                    [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0],
                    [0, 0, 0, 1]]
        else:
            continue
        m = _mat_mul(m, step)
    return m


def _collada_colors(root):
    effects = {}
    for effect in root.iter('effect'):
        for shading in ('phong', 'lambert', 'blinn', 'constant'):
            technique = effect.find('.//technique/' + shading)
            if technique is None:
                continue
            channel = technique.find('diffuse') if shading != 'constant' \
                else technique.find('emission')
            color = channel.find('color') if channel is not None else None
            if color is not None:
                values = [float(v) for v in color.text.split()]
                if len(values) >= 3:
                    # <transparency> is exported inconsistently between
                    # tools, so only the color's own alpha is used.
                    effects[effect.get('id')] = tuple((values + [1.0])[:4])
            break
    colors = {}
    for material in root.iter('material'):
        instance = material.find('instance_effect')
        if instance is not None:
            colors[material.get('id')] = effects.get(instance.get('url', '').lstrip('#'))
    return colors


def _collada_geometry(geometry):
    """Return {material_symbol: [triangles]} in the geometry's own frame."""
    mesh = geometry.find('mesh')
    if mesh is None:
        return {}
    sources = {}
    for source in mesh.findall('source'):
        array = source.find('float_array')
        accessor = source.find('technique_common/accessor')
        if array is None:
            continue
        stride = int(accessor.get('stride', 3)) if accessor is not None else 3
        sources[source.get('id')] = ([float(v) for v in (array.text or '').split()], stride)
    vertex_sources = {}
    for vertices in mesh.findall('vertices'):
        for item in vertices.findall('input'):
            if item.get('semantic') == 'POSITION':
                vertex_sources[vertices.get('id')] = item.get('source', '').lstrip('#')

    groups = {}
    for primitive in mesh:
        if primitive.tag not in ('triangles', 'polylist', 'polygons', 'trifans', 'tristrips'):
            continue
        inputs = primitive.findall('input')
        if not inputs:
            continue
        stride = max(int(i.get('offset', 0)) for i in inputs) + 1
        vertex_input = next((i for i in inputs if i.get('semantic') == 'VERTEX'), None)
        if vertex_input is None:
            continue
        offset = int(vertex_input.get('offset', 0))
        position_id = vertex_sources.get(vertex_input.get('source', '').lstrip('#'))
        if position_id not in sources:
            raise MeshError('Collada geometry "%s" has no position source' % geometry.get('id'))
        values, vstride = sources[position_id]

        def vertex(index):
            base = index * vstride
            if base + 3 > len(values):
                raise MeshError('Collada vertex index %d out of range' % index)
            return tuple(values[base:base + 3])

        polygons = []
        if primitive.tag == 'polylist':
            counts = [int(v) for v in (primitive.findtext('vcount') or '').split()]
            indices = [int(v) for v in (primitive.findtext('p') or '').split()]
            cursor = 0
            for count in counts:
                polygons.append([indices[(cursor + k) * stride + offset] for k in range(count)])
                cursor += count
        elif primitive.tag == 'triangles':
            indices = [int(v) for v in (primitive.findtext('p') or '').split()]
            corners = [indices[k * stride + offset] for k in range(len(indices) // stride)]
            polygons = [corners[k:k + 3] for k in range(0, len(corners) - 2, 3)]
        else:
            for p in primitive.findall('p'):
                indices = [int(v) for v in (p.text or '').split()]
                corners = [indices[k * stride + offset] for k in range(len(indices) // stride)]
                if primitive.tag == 'tristrips':
                    for k in range(len(corners) - 2):
                        tri = corners[k:k + 3]
                        polygons.append(tri if k % 2 == 0 else [tri[1], tri[0], tri[2]])
                else:
                    polygons.append(corners)
        faces = groups.setdefault(primitive.get('material'), [])
        for polygon in polygons:
            points = [vertex(i) for i in polygon]
            for k in range(1, len(points) - 1):
                faces.append((points[0], points[k], points[k + 1]))
    return groups


def read_dae(path):
    try:
        root = _strip_namespaces(ET.parse(path).getroot())
    except ET.ParseError as exc:
        raise MeshError('%s is not valid Collada XML: %s' % (path, exc))
    unit = root.find('asset/unit')
    meter = float(unit.get('meter', 1.0)) if unit is not None else 1.0
    up_axis = (root.findtext('asset/up_axis') or 'Y_UP').strip()
    up = _mat_identity()
    if up_axis == 'Y_UP':  # rotate +90 degrees about x: +y becomes +z
        up = [[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]]
    elif up_axis == 'X_UP':  # rotate -90 degrees about y: +x becomes +z
        up = [[0, 0, -1, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]]
    scale = [[meter if i == j and i < 3 else (1.0 if i == j else 0.0)
              for j in range(4)] for i in range(4)]
    base = _mat_mul(up, scale)

    geometries = {g.get('id'): g for g in root.iter('geometry')}
    library_nodes = {n.get('id'): n for lib in root.iter('library_nodes')
                     for n in lib.iter('node')}
    material_colors = _collada_colors(root)
    parts = {}

    def emit(geometry_id, matrix, bindings):
        geometry = geometries.get(geometry_id)
        if geometry is None:
            raise MeshError('%s references missing geometry "%s"' % (path, geometry_id))
        mirrored = _det3(matrix) < 0
        for symbol, faces in _collada_geometry(geometry).items():
            rgba = material_colors.get(bindings.get(symbol, symbol))
            out = parts.setdefault(rgba, [])
            for a, b, c in faces:
                a, b, c = (_mat_apply(matrix, v) for v in (a, b, c))
                out.append((a, c, b) if mirrored else (a, b, c))

    def walk(node, parent, depth=0):
        if depth > 64:
            raise MeshError('%s: Collada node hierarchy too deep or cyclic' % path)
        matrix = _mat_mul(parent, _node_matrix(node))
        for instance in node.findall('instance_geometry'):
            bindings = {m.get('symbol'): m.get('target', '').lstrip('#')
                        for m in instance.iter('instance_material')}
            emit(instance.get('url', '').lstrip('#'), matrix, bindings)
        for instance in node.findall('instance_node'):
            target = library_nodes.get(instance.get('url', '').lstrip('#'))
            if target is not None:
                walk(target, matrix, depth + 1)
        for child in node.findall('node'):
            walk(child, matrix, depth + 1)

    scenes = list(root.iter('visual_scene'))
    if scenes:
        wanted = root.find('scene/instance_visual_scene')
        scene = scenes[0]
        if wanted is not None:
            scene = next((s for s in scenes if s.get('id') == wanted.get('url', '').lstrip('#')),
                         scenes[0])
        for node in scene.findall('node'):
            walk(node, base)
    else:
        for geometry_id in geometries:
            emit(geometry_id, base, {})
    return [MeshPart(faces, rgba) for rgba, faces in parts.items() if faces]


READERS = {'.stl': read_stl, '.obj': read_obj, '.dae': read_dae}


def read_mesh(path):
    reader = READERS.get(os.path.splitext(path)[1].lower())
    if reader is None:
        raise MeshError('unsupported mesh format "%s" (supported: %s)'
                        % (os.path.basename(path), ', '.join(sorted(READERS))))
    parts = reader(path)
    if not any(part.triangles for part in parts):
        raise MeshError('%s contains no triangles' % path)
    return parts


def scale_triangles(triangles, scale):
    sx, sy, sz = scale
    mirrored = sx * sy * sz < 0
    out = []
    for tri in triangles:
        a, b, c = ((v[0] * sx, v[1] * sy, v[2] * sz) for v in tri)
        out.append((a, c, b) if mirrored else (a, b, c))
    return out


def bounds(triangles):
    points = [v for tri in triangles for v in tri]
    return (tuple(min(p[i] for p in points) for i in range(3)),
            tuple(max(p[i] for p in points) for i in range(3)))
