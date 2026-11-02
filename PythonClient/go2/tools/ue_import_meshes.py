"""Editor-side (Unreal Python) import of the robot link meshes + per-link materials.

Run inside the Unreal editor (Python console, or the MCP bridge's execute_python with file=...). Imports every
SM_*.obj of SRC_DIR into DEST (/AirSim/G1/Meshes), creates two flat materials (dark 0.2 / light 0.7, the rgba
Unitree gives the links in g1_12dof.xml) and assigns them, so the humanoid renders with Unitree's colour scheme.

Parameters are read from environment variables so the same file serves any robot:
  G1_OBJ_DIR   directory with the SM_*.obj files (default: the g1_obj_cm folder next to this script)
  G1_DEST      content path (default /AirSim/G1/Meshes)
  G1_BATCH     names to import in this call, comma separated (default: all) -- the MCP bridge times out after
               30 s, so import a handful per call
"""
import os
import json
import unreal

SRC = os.environ.get("G1_OBJ_DIR", os.path.join(os.path.dirname(os.path.abspath(__file__)), "g1_obj_cm"))
DEST = os.environ.get("G1_DEST", "/AirSim/G1/Meshes")
MAT_DIR = DEST.rsplit("/", 1)[0] + "/Materials"
BATCH = [s for s in os.environ.get("G1_BATCH", "").split(",") if s]

# rgba 0.2 links in g1_12dof.xml; everything else is 0.7
DARK = {"pelvis", "logo_link", "head_link", "left_hip_pitch_link", "right_hip_pitch_link", "left_ankle_roll_link", "right_ankle_roll_link"}


def ensure_material(name, grey):
    path = "%s/%s" % (MAT_DIR, name)
    if unreal.EditorAssetLibrary.does_asset_exist(path):
        return unreal.EditorAssetLibrary.load_asset(path)
    mat = unreal.AssetToolsHelpers.get_asset_tools().create_asset(name, MAT_DIR, unreal.Material, unreal.MaterialFactoryNew())
    mel = unreal.MaterialEditingLibrary
    c = mel.create_material_expression(mat, unreal.MaterialExpressionConstant3Vector, -300, 0)
    c.set_editor_property("constant", unreal.LinearColor(grey, grey, grey, 1.0))
    mel.connect_material_property(c, "", unreal.MaterialProperty.MP_BASE_COLOR)
    r = mel.create_material_expression(mat, unreal.MaterialExpressionConstant, -300, 150)
    r.set_editor_property("r", 0.55)
    mel.connect_material_property(r, "", unreal.MaterialProperty.MP_ROUGHNESS)
    mel.recompile_material(mat)
    unreal.EditorAssetLibrary.save_asset(path)
    return mat


def main():
    names = sorted(f[:-4] for f in os.listdir(SRC) if f.endswith(".obj"))
    if BATCH:
        names = [n for n in names if n in BATCH]
    tasks = []
    for n in names:
        t = unreal.AssetImportTask()
        t.set_editor_property("filename", os.path.join(SRC, n + ".obj"))
        t.set_editor_property("destination_path", DEST)
        t.set_editor_property("automated", True)
        t.set_editor_property("replace_existing", True)
        t.set_editor_property("save", True)
        tasks.append(t)
    if tasks:
        unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks(tasks)
    dark, light = ensure_material("M_g1_dark", 0.2), ensure_material("M_g1_light", 0.7)
    out = {}
    for n in names:
        p = "%s/%s" % (DEST, n)
        if not unreal.EditorAssetLibrary.does_asset_exist(p):
            out[n] = "MISSING"
            continue
        sm = unreal.EditorAssetLibrary.load_asset(p)
        link = n.split("_", 2)[2] if n.count("_") >= 2 else n   # SM_g1_<link>
        sm.set_material(0, dark if link in DARK else light)
        unreal.EditorAssetLibrary.save_asset(p)
        b = sm.get_bounding_box()
        out[n] = [round(b.min.x, 1), round(b.min.y, 1), round(b.min.z, 1), round(b.max.x, 1), round(b.max.y, 1), round(b.max.z, 1)]
    print(json.dumps(out))


main()
