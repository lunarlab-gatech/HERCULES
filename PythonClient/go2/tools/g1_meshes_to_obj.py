#!/usr/bin/env python3
"""Convert the Unitree G1 link meshes (STL, metres, from unitree_rl_gym resources/robots/g1_description) to OBJ files
in centimetres for the Unreal static-mesh importer, one per <mesh> of g1_12dof.xml, named SM_g1_<mesh name>.

Why: the Unreal OBJ importer mirrors Y (exactly the MuJoCo right-handed -> Unreal left-handed conversion) and keeps
Z up, so the only change needed is x100. Meshes above --max-tris are decimated (quadric, open3d): the raw torso has
157 k triangles and the hands 70 k each, far more than a 752x480 robot camera ever resolves, and the editor's OBJ
importer is very slow on them.

usage: g1_meshes_to_obj.py [--rl-gym ~/multi-robot-coordination/go2_thirdparty/unitree_rl_gym] [--out DIR] [--max-tris 4000]
then import DIR/*.obj into /AirSim/G1/Meshes with ue_import_meshes.py (run inside the editor).
"""
import argparse
import os
import re
import struct

import numpy as np


def read_stl(path):
    b = open(path, "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    a = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return a["v"].reshape(-1, 3, 3).astype(np.float64)


def write_obj(path, V, F, N, header):
    with open(path, "w") as o:
        o.write("# %s\n" % header)
        for v in V:
            o.write("v %.5f %.5f %.5f\n" % tuple(v))
        for v in N:
            o.write("vn %.4f %.4f %.4f\n" % tuple(v))
        for t in F:
            o.write("f %d//%d %d//%d %d//%d\n" % (t[0] + 1, t[0] + 1, t[1] + 1, t[1] + 1, t[2] + 1, t[2] + 1))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rl-gym", default=os.path.expanduser("~/multi-robot-coordination/go2_thirdparty/unitree_rl_gym"))
    ap.add_argument("--out", default="g1_obj_cm")
    ap.add_argument("--max-tris", type=int, default=4000)
    a = ap.parse_args()
    import open3d as o3d
    desc = os.path.join(a.rl_gym, "resources/robots/g1_description")
    meshes = re.findall(r'<mesh name="([^"]+)" file="([^"]+)"', open(os.path.join(desc, "g1_12dof.xml")).read())
    os.makedirs(a.out, exist_ok=True)
    for name, fn in meshes:
        tri = read_stl(os.path.join(desc, "meshes", fn)) * 100.0
        v = tri.reshape(-1, 3)
        uv, inv = np.unique(np.round(v, 5), axis=0, return_inverse=True)
        m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(uv), o3d.utility.Vector3iVector(inv.reshape(-1, 3)))
        n0 = len(m.triangles)
        if n0 > a.max_tris:
            m = m.simplify_quadric_decimation(target_number_of_triangles=a.max_tris)
        m.remove_degenerate_triangles(); m.remove_duplicated_vertices(); m.remove_unreferenced_vertices()
        m.compute_vertex_normals()
        V, F, N = np.asarray(m.vertices), np.asarray(m.triangles), np.asarray(m.vertex_normals)
        out = os.path.join(a.out, "SM_g1_%s.obj" % name)
        write_obj(out, V, F, N, "unitree g1 %s (%s), cm, %d -> %d tris" % (name, fn, n0, len(F)))
        print("%-32s %7d -> %5d tris -> %s" % (name, n0, len(F), out))


if __name__ == "__main__":
    main()
