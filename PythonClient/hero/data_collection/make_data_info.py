#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate DAIR-V2X style data_info.json files:

- <ROOT>/cooperative/data_info.json            (infra/veh img+pcd, coop label_world, system_error_offset)
- <ROOT>/vehicle-side/data_info.json           (img/pcd timestamps, calib_*novatel*, label/lidar)
- <ROOT>/infrastructure-side/data_info.json    (img/pcd timestamps, calib_*virtuallidar*, label/virtuallidar)

If a required per-frame calibration json is missing, this script writes an identity JSON so
downstream code can open the path instead of crashing on None.
"""

from pathlib import Path
import json

# =========================
# USER SETTINGS (edit me!)
# =========================
ROOT = Path("/home/dellg16ssg/multi-robot-coordination/collaborative-perception-BEVP/datasets/dair_v2x_synth_FULL/cooperative-vehicle-infrastructure")

# subfolders inside each side
IMAGE_DIR_NAME  = "image"
LIDAR_DIR_NAME  = "velodyne"

# cooperative labels (expected location)
COOP_LABEL_DIR  = "cooperative/label_world"

# extensions to accept (order = preference)
IMG_EXTS   = (".jpg", ".png", ".jpeg")
LIDAR_EXTS = (".pcd", ".bin", ".pvd")  # original uses .pcd

# ---- per-side folder names for labels & calibs (match original dataset) ----
# vehicle-side
VEH_LABEL_LIDAR_DIR    = "label/lidar"
VEH_LABEL_CAMERA_DIR   = "label/camera"
VEH_CALIB_INTR_DIR     = "calib/camera_intrinsic"
VEH_CALIB_L2CAM_DIR    = "calib/lidar_to_camera"
VEH_CALIB_L2NOV_DIR    = "calib/lidar_to_novatel"
VEH_CALIB_NOV2W_DIR    = "calib/novatel_to_world"

# infrastructure-side
INF_LABEL_VIRTLIDAR_DIR  = "label/virtuallidar"
INF_LABEL_CAMERA_DIR     = "label/camera"
INF_CALIB_INTR_DIR       = "calib/camera_intrinsic"
INF_CALIB_VL2CAM_DIR     = "calib/virtuallidar_to_camera"
INF_CALIB_VL2W_DIR       = "calib/virtuallidar_to_world"

# identity/default calibration values (used only when a per-frame json is missing)
DEFAULT_FX, DEFAULT_FY, DEFAULT_CX, DEFAULT_CY = 1000.0, 1000.0, 960.0, 540.0
DEFAULT_WIDTH, DEFAULT_HEIGHT = 1920, 1080
DEFAULT_DISTORTION = [0, 0, 0, 0, 0]
I3 = [[1,0,0],[0,1,0],[0,0,1]]
Z3 = [0,0,0]

# Default timestamps/batching
DEFAULT_IMAGE_TIMESTAMP      = ""   # e.g., "1626155123061000"
DEFAULT_POINTCLOUD_TIMESTAMP = ""   # e.g., "1626155122981522"
DEFAULT_BATCH_ID             = "0"
DEFAULT_BATCH_START_ID       = None # if None -> use frame_id
DEFAULT_BATCH_END_ID         = None # if None -> use frame_id
DEFAULT_INTERSECTION_LOC     = ""

# =========================
# helpers
# =========================
def list_ids(d: Path, exts):
    ids = set()
    if d.exists():
        for p in d.iterdir():
            if p.is_file() and p.suffix.lower() in exts:
                ids.add(p.stem)
    return ids

def first_existing(side_root: Path, rel_paths):
    for rel in rel_paths:
        if rel and (side_root / rel).exists():
            return rel
    return None

def ensure_json(path: Path, payload: dict):
    if path.exists():
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

def rel_exists_under_root(root: Path, rel: str) -> bool:
    """Check that a relative path exists under ROOT (guards against accidental directory checks)."""
    if not rel:
        return False
    return (root / rel).is_file()

# ----- identity writers for missing calibration -----
def ensure_vehicle_calibs(side_root: Path, fid: str):
    # intrinsics
    ensure_json(
        side_root / f"{VEH_CALIB_INTR_DIR}/{fid}.json",
        {
            "camera_intrinsic": [
                [DEFAULT_FX, 0, DEFAULT_CX],
                [0, DEFAULT_FY, DEFAULT_CY],
                [0, 0, 1],
            ],
            "distortion": DEFAULT_DISTORTION,
            "width_height": [DEFAULT_WIDTH, DEFAULT_HEIGHT],
        },
    )
    # lidar->camera
    ensure_json(
        side_root / f"{VEH_CALIB_L2CAM_DIR}/{fid}.json",
        {"rotation": I3, "translation": Z3},
    )
    # lidar->novatel
    ensure_json(
        side_root / f"{VEH_CALIB_L2NOV_DIR}/{fid}.json",
        {"rotation": I3, "translation": Z3},
    )
    # novatel->world
    ensure_json(
        side_root / f"{VEH_CALIB_NOV2W_DIR}/{fid}.json",
        {"rotation": I3, "translation": Z3},
    )

def ensure_infra_calibs(side_root: Path, fid: str):
    # intrinsics
    ensure_json(
        side_root / f"{INF_CALIB_INTR_DIR}/{fid}.json",
        {
            "camera_intrinsic": [
                [DEFAULT_FX, 0, DEFAULT_CX],
                [0, DEFAULT_FY, DEFAULT_CY],
                [0, 0, 1],
            ],
            "distortion": DEFAULT_DISTORTION,
            "width_height": [DEFAULT_WIDTH, DEFAULT_HEIGHT],
        },
    )
    # virtuallidar->camera
    ensure_json(
        side_root / f"{INF_CALIB_VL2CAM_DIR}/{fid}.json",
        {"rotation": I3, "translation": Z3},
    )
    # virtuallidar->world
    ensure_json(
        side_root / f"{INF_CALIB_VL2W_DIR}/{fid}.json",
        {"rotation": I3, "translation": Z3},
    )

# =========================
# per-side writers
# =========================
def build_vehicle_side_entries(side_root: Path, ids):
    out = []
    for fid in sorted(ids):
        img_rel = first_existing(side_root, [f"{IMAGE_DIR_NAME}/{fid}{ext}" for ext in IMG_EXTS])
        pcd_rel = first_existing(side_root,   [f"{LIDAR_DIR_NAME}/{fid}{ext}" for ext in LIDAR_EXTS])
        if not img_rel and not pcd_rel:
            continue  # no data → skip

        # ensure per-frame calibs exist so paths are valid strings
        ensure_vehicle_calibs(side_root, fid)

        entry = {
            "image_path": img_rel or "",
            "image_timestamp": DEFAULT_IMAGE_TIMESTAMP,
            "pointcloud_path": pcd_rel or "",
            "pointcloud_timestamp": DEFAULT_POINTCLOUD_TIMESTAMP,
            "calib_novatel_to_world_path": f"{VEH_CALIB_NOV2W_DIR}/{fid}.json",
            "calib_lidar_to_novatel_path": f"{VEH_CALIB_L2NOV_DIR}/{fid}.json",
            "calib_lidar_to_camera_path": f"{VEH_CALIB_L2CAM_DIR}/{fid}.json",
            "calib_camera_intrinsic_path": f"{VEH_CALIB_INTR_DIR}/{fid}.json",
            "label_camera_std_path": f"{VEH_LABEL_CAMERA_DIR}/{fid}.json" if (side_root / f"{VEH_LABEL_CAMERA_DIR}/{fid}.json").exists() else "",
            "label_lidar_std_path":  f"{VEH_LABEL_LIDAR_DIR}/{fid}.json"  if (side_root / f"{VEH_LABEL_LIDAR_DIR}/{fid}.json").exists()  else "",
            "batch_start_id": (DEFAULT_BATCH_START_ID or fid),
            "batch_end_id":   (DEFAULT_BATCH_END_ID   or fid),
            "intersection_loc": DEFAULT_INTERSECTION_LOC,
            "batch_id": DEFAULT_BATCH_ID,
        }
        out.append(entry)
    return out

def build_infra_side_entries(side_root: Path, ids):
    out = []
    for fid in sorted(ids):
        img_rel = first_existing(side_root, [f"{IMAGE_DIR_NAME}/{fid}{ext}" for ext in IMG_EXTS])
        pcd_rel = first_existing(side_root,   [f"{LIDAR_DIR_NAME}/{fid}{ext}" for ext in LIDAR_EXTS])
        if not img_rel and not pcd_rel:
            continue

        # ensure per-frame calibs exist so paths are valid strings
        ensure_infra_calibs(side_root, fid)

        entry = {
            "pointcloud_path": pcd_rel or "",
            "pointcloud_timestamp": DEFAULT_POINTCLOUD_TIMESTAMP,
            "lidar_id": "",
            "intersection_loc": DEFAULT_INTERSECTION_LOC,
            "batch_start_id": (DEFAULT_BATCH_START_ID or fid),
            "batch_end_id":   (DEFAULT_BATCH_END_ID   or fid),
            "calib_camera_intrinsic_path":      f"{INF_CALIB_INTR_DIR}/{fid}.json",
            "calib_virtuallidar_to_world_path": f"{INF_CALIB_VL2W_DIR}/{fid}.json",
            "calib_virtuallidar_to_camera_path":f"{INF_CALIB_VL2CAM_DIR}/{fid}.json",
            "label_lidar_std_path":  f"{INF_LABEL_VIRTLIDAR_DIR}/{fid}.json" if (side_root / f"{INF_LABEL_VIRTLIDAR_DIR}/{fid}.json").exists() else "",
            "image_path": img_rel or "",
            "image_timestamp": DEFAULT_IMAGE_TIMESTAMP,
            "label_camera_std_path": f"{INF_LABEL_CAMERA_DIR}/{fid}.json" if (side_root / f"{INF_LABEL_CAMERA_DIR}/{fid}.json").exists() else "",
            "camera_ip": "",
            "camera_id": "",
            "batch_id": DEFAULT_BATCH_ID,
            "valid_batch_splits": [{
                "batch_start_id": (DEFAULT_BATCH_START_ID or fid),
                "batch_end_id":   (DEFAULT_BATCH_END_ID   or fid),
            }],
        }
        out.append(entry)
    return out

# =========================
# cooperative writer
# =========================
def build_cooperative_entries(root: Path, infra_ids, veh_ids):
    infra = root / "infrastructure-side"
    veh   = root / "vehicle-side"
    pairs = []

    missing_coop = []
    made = 0

    # Pair by SAME id present on both sides
    common = sorted(infra_ids & veh_ids)
    for fid in common:
        inf_img = first_existing(infra, [f"{IMAGE_DIR_NAME}/{fid}{ext}" for ext in IMG_EXTS])
        veh_img = first_existing(veh,   [f"{IMAGE_DIR_NAME}/{fid}{ext}" for ext in IMG_EXTS])
        inf_pcd = first_existing(infra, [f"{LIDAR_DIR_NAME}/{fid}{ext}" for ext in LIDAR_EXTS])
        veh_pcd = first_existing(veh,   [f"{LIDAR_DIR_NAME}/{fid}{ext}" for ext in LIDAR_EXTS])
        if not (inf_img and veh_img):
            continue

        # expected coop label
        coop_rel = f"{COOP_LABEL_DIR}/{fid}.json"
        coop_ok  = rel_exists_under_root(root, coop_rel)

        if not coop_ok:
            missing_coop.append(coop_rel)
            # Still write the entry but leave field blank (matches original behavior)
            coop_rel_out = ""
        else:
            coop_rel_out = coop_rel
            made += 1

        pairs.append({
            "infrastructure_image_path":      f"infrastructure-side/{inf_img}",
            "infrastructure_pointcloud_path": f"infrastructure-side/{inf_pcd}" if inf_pcd else "",
            "vehicle_image_path":             f"vehicle-side/{veh_img}",
            "vehicle_pointcloud_path":        f"vehicle-side/{veh_pcd}" if veh_pcd else "",
            "cooperative_label_path":         coop_rel_out,
            "system_error_offset": {"delta_x": 0.0, "delta_y": 0.0},
        })

    # concise diagnostics
    print(f"[cooperative] total paired by same-id: {len(common)}")
    print(f"[cooperative] cooperative labels found: {made}  missing: {len(missing_coop)}")
    if missing_coop[:10]:
        print("[cooperative] first missing examples:")
        for m in missing_coop[:10]:
            print("  -", m)

    return pairs

# =========================
# main
# =========================
def main():
    infra = ROOT / "infrastructure-side"
    veh   = ROOT / "vehicle-side"
    coop  = ROOT / "cooperative"
    coop.mkdir(parents=True, exist_ok=True)

    infra_ids = list_ids(infra / IMAGE_DIR_NAME, IMG_EXTS)
    veh_ids   = list_ids(veh   / IMAGE_DIR_NAME, IMG_EXTS)

    veh_entries  = build_vehicle_side_entries(veh,   veh_ids)
    inf_entries  = build_infra_side_entries(infra,  infra_ids)
    coop_entries = build_cooperative_entries(ROOT, infra_ids, veh_ids)

    # write
    (veh   / "data_info.json").write_text(json.dumps(veh_entries,  indent=2), encoding="utf-8")
    (infra / "data_info.json").write_text(json.dumps(inf_entries,  indent=2), encoding="utf-8")
    (coop  / "data_info.json").write_text(json.dumps(coop_entries, indent=2), encoding="utf-8")

    print(f"[scan] veh images: {len(veh_ids)}  infra images: {len(infra_ids)}  coop pairs: {len(coop_entries)}")
    print(f"[write] {veh/'data_info.json'}")
    print(f"[write] {infra/'data_info.json'}")
    print(f"[write] {coop/'data_info.json'}")

if __name__ == "__main__":
    main()