import math
import os
import struct

from conftest import DESCRIPTION
from hercules_urdf_import import meshes
import pytest

MESHES = os.path.join(DESCRIPTION, 'meshes')


def signed_volume(triangles):
    """Positive for a closed mesh whose faces wind CCW seen from outside."""
    total = 0.0
    for a, b, c in triangles:
        total += (a[0] * (b[1] * c[2] - b[2] * c[1])
                  - a[1] * (b[0] * c[2] - b[2] * c[0])
                  + a[2] * (b[0] * c[1] - b[1] * c[0]))
    return total / 6.0


def edges_are_closed(triangles):
    """Every directed edge is matched by its reverse (watertight, consistent)."""
    def key(p):
        return tuple(round(c, 9) for c in p)
    directed = {}
    for tri in triangles:
        for i in range(3):
            edge = (key(tri[i]), key(tri[(i + 1) % 3]))
            directed[edge] = directed.get(edge, 0) + 1
    return all(directed.get((b, a), 0) == n for (a, b), n in directed.items())


def test_box_is_closed_outward_and_sized():
    tris = meshes.box((1.0, 2.0, 3.0))
    assert len(tris) == 12
    assert edges_are_closed(tris)
    assert signed_volume(tris) == pytest.approx(6.0)
    assert meshes.bounds(tris) == ((-0.5, -1.0, -1.5), (0.5, 1.0, 1.5))


def test_cylinder_is_closed_outward_along_z():
    tris = meshes.cylinder(0.5, 2.0, segments=64)
    assert edges_are_closed(tris)
    polygon_area = 0.5 * 64 * 0.25 * math.sin(2 * math.pi / 64)
    assert signed_volume(tris) == pytest.approx(polygon_area * 2.0)
    lo, hi = meshes.bounds(tris)
    assert lo[2] == pytest.approx(-1.0) and hi[2] == pytest.approx(1.0)


def test_sphere_is_closed_and_outward():
    tris = meshes.sphere(1.0)
    assert edges_are_closed(tris)
    assert 0.9 * 4 / 3 * math.pi < signed_volume(tris) < 4 / 3 * math.pi


def test_mirrored_scale_keeps_faces_outward():
    tris = meshes.scale_triangles(meshes.box((1, 1, 1)), (2.0, -1.0, 1.0))
    assert signed_volume(tris) == pytest.approx(2.0)


def test_binary_stl_round_trip(tmp_path):
    tris = meshes.box((0.2, 0.3, 0.4))
    path = str(tmp_path / 'box.stl')
    meshes.write_binary_stl(path, tris)
    assert os.path.getsize(path) == 84 + 50 * len(tris)
    (part,) = meshes.read_stl(path)
    assert signed_volume(part.triangles) == pytest.approx(0.024, rel=1e-6)
    # stored facet normals point outward
    with open(path, 'rb') as stream:
        data = stream.read()
    for i, (a, b, c) in enumerate(tris):
        normal = struct.unpack_from('<3f', data, 84 + 50 * i)
        centroid = [sum(v[k] for v in (a, b, c)) / 3 for k in range(3)]
        assert sum(n * x for n, x in zip(normal, centroid)) > 0


def test_binary_stl_whose_header_starts_with_solid():
    (part,) = meshes.read_stl(os.path.join(MESHES, 'top_plate.stl'))
    assert len(part.triangles) == 12
    assert signed_volume(part.triangles) == pytest.approx(0.6 * 0.4 * 0.02, rel=1e-5)


def test_ascii_stl():
    (part,) = meshes.read_stl(os.path.join(MESHES, 'antenna.stl'))
    assert len(part.triangles) == 2
    assert part.triangles[0][2] == (0.0, 0.0, 0.05)


def test_obj_groups_by_material_with_mtl_colors():
    parts = meshes.read_obj(os.path.join(MESHES, 'mast.obj'))
    colors = sorted(p.rgba for p in parts)
    assert colors == [(0.1, 0.2, 0.9, 1.0), (1.0, 1.0, 1.0, 1.0)]
    # each quad (one positive, one negative-index face) is fanned into 2 triangles
    assert [len(p.triangles) for p in parts] == [2, 2]


def test_collada_units_up_axis_mirroring_and_color():
    (part,) = meshes.read_dae(os.path.join(MESHES, 'bumper.dae'))
    assert part.rgba == (0.8, 0.1, 0.1, 1.0)
    assert len(part.triangles) == 8  # 3 triangles + 1 polylist face, two instances
    # centimeters -> meters and Y_UP -> Z_UP: the 10 cm "up" (y) vertex is now +z
    lo, hi = meshes.bounds(part.triangles)
    assert hi[2] == pytest.approx(0.1)
    assert lo[1] == pytest.approx(-0.1)
    assert lo[0] == pytest.approx(-0.3) and hi[0] == pytest.approx(0.1)
    # both instances, including the mirrored one, stay outward-facing
    left = [t for t in part.triangles if max(v[0] for v in t) > 0.05 or
            min(v[0] for v in t) >= 0]
    right = [t for t in part.triangles if t not in left]
    tet = 0.1 ** 3 / 6
    assert signed_volume(left) == pytest.approx(tet)
    assert signed_volume(right) == pytest.approx(tet)


def test_unknown_format_is_rejected(tmp_path):
    path = tmp_path / 'robot.glb'
    path.write_bytes(b'glTF')
    with pytest.raises(meshes.MeshError, match='unsupported mesh format'):
        meshes.read_mesh(str(path))


def test_garbage_stl_is_rejected(tmp_path):
    path = tmp_path / 'bad.stl'
    path.write_bytes(b'\x00' * 10)
    with pytest.raises(meshes.MeshError):
        meshes.read_stl(str(path))
