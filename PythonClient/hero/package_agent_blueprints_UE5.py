"""Editor-side (Unreal Python) migration + audit + load test of the HERCULES dynamic-agent Blueprints.

Run headless:
  UnrealEditor-Cmd <HerculesAgentPkg.uproject> -run=pythonscript -script=<this file> -unattended -nopause -nosplash -NullRHI

Modes (env HPKG_MODE):
  package : rename the Blueprints out of their source folders into /Game/HERCULES/Agents/{Animals,Pedestrians,Vehicles},
            delete the redirectors left behind, save, then audit + compile (same as verify).
  verify  : audit every asset under /Game/HERCULES (direct dependencies classified as HERCULES / engine / third-party pack),
            load + compile each Blueprint, dump variables + components, write the manifest JSON (env HPKG_OUT).
"""
import json
import os
import re
import time
import traceback

import unreal

MODE = os.environ.get("HPKG_MODE", "verify")
OUT = os.environ.get("HPKG_OUT", "/tmp/hpkg_manifest.json")
ROOT = "/Game/HERCULES/Agents"
SRC_ANIMALS = "/Game/StarterContent/Blueprints/Animals"
SRC_CITY = "/Game/CustomHumanPedestrians"
VEHICLE_NAMES = ("BP_SplineRoad", "BP_SplineVehicle")
# Blueprint variables found in the assets' name tables (UBlueprint.NewVariables is not exposed to Python in 5.2),
# read back from each class default object so the docs can quote real defaults.
CANDIDATE_VARS = ["Radius", "AcceptanceRadius", "MaxWalkSpeed", "YawSpeed", "SplineSpeed", "DistanceAlongSpline",
                  "SplineToFollow", "Reverse", "ReverseFromEnd", "MinSpeed", "MaxSpeed", "AdditionalOffset",
                  "DebugSteering", "DebugBrake", "ai_controller_class", "auto_possess_ai",
                  # variables whose FName contains spaces (Blueprint variable names may)
                  "Spline Speed", "Distance Along Spline", "Spline To Follow", "Yaw Speed", "Reverse From End"]

log = unreal.log
warn = unreal.log_warning
ar = unreal.AssetRegistryHelpers.get_asset_registry()
eal = unreal.EditorAssetLibrary


def pkg_of(obj_path):
    return obj_path.split(".")[0]


def dest_for(pkg):
    name = pkg.rsplit("/", 1)[1]
    if pkg.startswith(SRC_ANIMALS + "/"):
        return "%s/Animals/%s" % (ROOT, name)
    if pkg.startswith(SRC_CITY + "/"):
        if name.startswith("BP_VehicleAI") or name in VEHICLE_NAMES:
            return "%s/Vehicles/%s" % (ROOT, name)
        return "%s/Pedestrians/%s" % (ROOT, name)
    return None


def list_pkgs(folder):
    return sorted(set(pkg_of(p) for p in (eal.list_assets(folder, recursive=True, include_folder=False) or [])))


BASES_FIRST = ("BP_Animal", "BP_SplineHuman", "BP_SplinePath", "BP_SplineRoad", "BP_VehicleAI")
OLD_PREFIXES = (b"/Game/CustomHumanPedestrians", b"/Game/StarterContent/Blueprints/Animals")


def content_dir():
    return os.path.normpath(os.path.join(unreal.SystemLibrary.get_project_content_directory()))


def file_audit(label):
    """Byte-level scan of every saved HERCULES asset for the old package paths (independent of the asset registry)."""
    root = os.path.join(content_dir(), "HERCULES")
    bad = []
    for dp, _, fns in os.walk(root):
        for fn in fns:
            if fn.endswith(".uasset"):
                data = open(os.path.join(dp, fn), "rb").read()
                hits = [pfx.decode() for pfx in OLD_PREFIXES if pfx in data]
                if hits:
                    bad.append((fn, hits))
    log("[hpkg] file audit (%s): %d files with stale paths %s" % (label, len(bad), bad))
    return bad


def do_package():
    """Idempotent: moves whatever still sits in the source folders (skipping redirectors from an earlier run)."""
    moves = []
    for folder in (SRC_ANIMALS, SRC_CITY):
        for obj in (eal.list_assets(folder, recursive=True, include_folder=False) or []):
            ad = eal.find_asset_data(obj)
            if ad and str(ad.asset_class_path.asset_name) == "ObjectRedirector":
                continue
            dst = dest_for(pkg_of(obj))
            if dst:
                moves.append((pkg_of(obj), dst))
    # parents / referenced classes first, so every child is saved with the final path of its base already in place
    moves.sort(key=lambda m: (0 if m[0].rsplit("/", 1)[1] in BASES_FIRST else 1, m[0]))
    log("[hpkg] %d assets to move" % len(moves))
    for src, dst in moves:
        ok = eal.rename_asset(src, dst)
        log("[hpkg] rename %s -> %s : %s" % (src, dst, ok))
        if not ok:
            raise RuntimeError("rename failed: " + src)
    ar.scan_paths_synchronous([SRC_ANIMALS, SRC_CITY, ROOT], force_rescan=True)
    # Load everything (redirectors still in place, so every reference resolves) and force-save: the import tables
    # are rebuilt from the live object references, i.e. the final paths.
    for pkg in list_pkgs(ROOT):
        if eal.load_asset(pkg) is None:
            raise RuntimeError("load failed after rename: " + pkg)
    for pkg in list_pkgs(ROOT):
        if not eal.save_asset(pkg, only_if_is_dirty=False):
            raise RuntimeError("save failed: " + pkg)
    if file_audit("after force-save"):
        raise RuntimeError("stale paths survived the force-save")
    # Redirectors left behind: UE keeps one when some package it did not re-save still points at the old path.
    # Here that can only be third-party content (e.g. the City Sample demo map the city BPs were dropped into),
    # which is not shipped, so the redirector is deleted unless one of OUR assets still goes through it.
    ar.scan_paths_synchronous([SRC_ANIMALS, SRC_CITY, ROOT], force_rescan=True)
    for folder in (SRC_ANIMALS, SRC_CITY):
        for obj in (eal.list_assets(folder, recursive=True, include_folder=False) or []):
            ad = eal.find_asset_data(obj)
            cls = str(ad.asset_class_path.asset_name) if ad else "?"
            refs = [str(r) for r in (ar.get_referencers(unreal.Name(pkg_of(obj)), unreal.AssetRegistryDependencyOptions()) or [])]
            ours = [r for r in refs if r.startswith(ROOT + "/")]
            log("[hpkg] leftover %s (%s) referenced by %s" % (obj, cls, refs))
            if ours:
                raise RuntimeError("leftover still referenced by a HERCULES asset: %s <- %s" % (obj, ours))
            if cls != "ObjectRedirector":
                raise RuntimeError("unexpected non-redirector leftover: " + obj)
            log("[hpkg] delete redirector %s : %s" % (obj, eal.delete_asset(obj)))
    ar.scan_paths_synchronous([ROOT], force_rescan=True)
    if file_audit("after redirector cleanup"):
        raise RuntimeError("stale paths after cleanup")


def classify(dep):
    d = str(dep)
    if d.startswith(ROOT + "/"):
        return "hercules"
    if d.startswith("/Script/") or d.startswith("/Engine/"):
        return "engine"
    if d.startswith("/Game/"):
        return "thirdparty:" + d.split("/")[2]
    return "other"


def deps_of(pkg, hard, soft):
    opts = unreal.AssetRegistryDependencyOptions(include_soft_package_references=soft, include_hard_package_references=hard,
                                                 include_searchable_names=False, include_soft_management_references=False,
                                                 include_hard_management_references=False)
    return sorted(str(d) for d in (ar.get_dependencies(unreal.Name(pkg), opts) or []))


def bp_variables(bp):
    out = []
    try:
        for v in bp.get_editor_property("new_variables") or []:
            try:
                flags = int(v.get_editor_property("property_flags"))
            except Exception:
                flags = -1
            try:
                vtype = v.get_editor_property("var_type")
                tname = str(vtype.get_editor_property("pin_category"))
                sub = vtype.get_editor_property("pin_sub_category_object")
                if sub:
                    tname += ":" + sub.get_name()
            except Exception:
                tname = "?"
            out.append({"name": str(v.get_editor_property("var_name")), "type": tname,
                        "default": str(v.get_editor_property("default_value")),
                        "category": str(v.get_editor_property("category")),
                        "instance_editable": (flags & 0x1 != 0 and flags & 0x10 == 0) if flags >= 0 else None,
                        "flags": flags})
    except Exception as e:
        out.append({"error": "new_variables not readable: %s" % e})
    return out


def cdo_values(bp, names):
    vals = {}
    try:
        cdo = unreal.get_default_object(bp.generated_class())
        for n in names:
            try:
                vals[n] = str(cdo.get_editor_property(n))
            except Exception:
                pass
    except Exception as e:
        vals["_error"] = str(e)
    return vals


def component_report(bp):
    comps = []
    try:
        world = unreal.EditorLevelLibrary.get_editor_world()
        if world is None:
            return [{"error": "no editor world"}]
        actor = unreal.EditorLevelLibrary.spawn_actor_from_class(bp.generated_class(), unreal.Vector(0, 0, 0))
        if actor is None:
            return [{"error": "spawn failed"}]
        for c in actor.get_components_by_class(unreal.ActorComponent):
            e = {"name": c.get_name(), "class": c.get_class().get_name()}
            for prop in ("skeletal_mesh_asset", "static_mesh", "anim_class", "animation_mode", "anim_to_play"):
                try:
                    v = c.get_editor_property(prop)
                    e[prop] = v.get_path_name() if hasattr(v, "get_path_name") else str(v)
                except Exception:
                    pass
            if isinstance(c, unreal.SplineComponent):
                e["spline_points"] = c.get_number_of_spline_points()
                e["spline_length_cm"] = c.get_spline_length()
            if isinstance(c, unreal.CharacterMovementComponent):
                e["max_walk_speed"] = c.get_editor_property("max_walk_speed")
            if isinstance(c, unreal.ChaosWheeledVehicleMovementComponent):
                try:
                    e["wheel_setups"] = len(c.get_editor_property("wheel_setups"))
                except Exception:
                    pass
            comps.append(e)
        actor.destroy_actor()
    except Exception as e:
        comps.append({"error": str(e)})
    return comps


def do_verify():
    ar.scan_paths_synchronous([ROOT], force_rescan=True)
    pkgs = list_pkgs(ROOT)
    log("[hpkg] verifying %d assets under %s" % (len(pkgs), ROOT))
    manifest_audit = file_audit("verify")
    manifest = {"generated": time.strftime("%Y-%m-%d %H:%M:%S"), "mode": MODE, "root": ROOT, "assets": {}}
    for pkg in pkgs:
        entry = {"load_ok": False}
        hard = deps_of(pkg, True, False)
        soft = deps_of(pkg, False, True)
        entry["deps_hard"] = {d: classify(d) for d in hard}
        entry["deps_soft"] = {d: classify(d) for d in soft}
        ad = eal.find_asset_data(pkg)
        if ad:
            entry["asset_class"] = str(ad.asset_class_path.asset_name)
            for tag in ("ParentClass", "NativeParentClass", "BlueprintType"):
                try:
                    entry[tag] = str(ad.get_tag_value(tag))
                except Exception:
                    pass
        try:
            bp = eal.load_asset(pkg)
            entry["load_ok"] = bp is not None
            if isinstance(bp, unreal.Blueprint):
                # A Blueprint whose parent class lives in a pack that is not installed loads (empty mesh, "Failed to load"
                # warning) but has a NULL parent: compiling / spawning it crashes the commandlet, and the editor itself
                # refuses to place such an actor, so record the condition and skip those two probes.
                parent_tag = entry.get("ParentClass", "")
                parent_path = parent_tag.split("'")[1] if "'" in parent_tag else parent_tag
                parent_ok = parent_path.startswith("/Script/") or unreal.load_object(None, parent_path) is not None
                entry["parent_loaded"] = parent_ok
                if not parent_ok:
                    entry["status"] = "parent class missing (%s): compile/spawn skipped" % parent_path
                    manifest["assets"][pkg] = entry
                    log("[hpkg] %s load=%s PARENT MISSING %s" % (pkg, entry["load_ok"], parent_path))
                    continue
                try:
                    unreal.BlueprintEditorLibrary.compile_blueprint(bp)
                except Exception as e:
                    entry["compile_error"] = str(e)
                try:
                    entry["status"] = str(bp.get_editor_property("status"))
                except Exception as e:
                    entry["status"] = "unreadable: %s" % e
                entry["variables"] = bp_variables(bp)
                names = [v["name"] for v in entry["variables"] if "name" in v] + CANDIDATE_VARS
                entry["cdo_values"] = cdo_values(bp, names)
                entry["components"] = component_report(bp)
            else:
                entry["class"] = bp.get_class().get_name() if bp else None
        except Exception as e:
            entry["load_error"] = str(e)
            entry["trace"] = traceback.format_exc()
        manifest["assets"][pkg] = entry
        log("[hpkg] %s load=%s status=%s hard=%d soft=%d" % (pkg, entry["load_ok"], entry.get("status"), len(hard), len(soft)))
    manifest["file_audit_stale"] = manifest_audit
    with open(OUT, "w") as f:
        json.dump(manifest, f, indent=1, sort_keys=True)
    log("[hpkg] manifest written: " + OUT)


try:
    if MODE == "package":
        do_package()
    do_verify()
    log("[hpkg] DONE OK")
except Exception:
    unreal.log_error("[hpkg] FAILED\n" + traceback.format_exc())
    raise
