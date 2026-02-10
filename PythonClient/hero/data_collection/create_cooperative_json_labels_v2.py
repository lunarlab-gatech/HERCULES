#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
import json
import numpy as np

# ==================== EDIT THIS PATH ====================
# ROOT = Path("/home/sgarimella34/multi-robot-coordination/collaborative-perception-BEVP/datasets/dair_v2x_synth_TEST1/cooperative-vehicle-infrastructure")
ROOT = Path("/home/dellg16ssg/multi-robot-coordination/collaborative-perception-BEVP/datasets/dair_v2x_synth_FULL/cooperative-vehicle-infrastructure/")
# ROOT = Path("/home/sgarimella34/multi-robot-coordination/collaborative-perception-BEVP/datasets/DAIR-V2X-C-SUBSET1/cooperative-vehicle-infrastructure/")

# ========================================================

# Inputs
VEH_LABEL_DIR     = ROOT / "vehicle-side/label/lidar"
VEH_CAL_L2NOV     = ROOT / "vehicle-side/calib/lidar_to_novatel"
VEH_CAL_NOV2W     = ROOT / "vehicle-side/calib/novatel_to_world"

INF_LABEL_DIR     = ROOT / "infrastructure-side/label/lidar"
INF_CAL_VL2W      = ROOT / "infrastructure-side/calib/virtuallidar_to_world"
INF_CAL_VL2BASE   = ROOT / "infrastructure-side/calib/virtuallidar_to_base"
INF_CAL_BASE2W    = ROOT / "infrastructure-side/calib/base_to_world"

# Output (merged vehicle + infrastructure, in WORLD frame)
OUT_DIR           = ROOT / "cooperative/label_world"

# ---------- DEDUP TUNING ----------
PREFER_ON_DUPLICATE = "vehicle"   # "vehicle" | "infra" (who wins when overlapping)
IOU3D_THRESH        = 0.25        # oriented 3D IoU threshold to consider as duplicate
IOU_BEV_THRESH      = 0.50        # fallback: oriented BEV IoU threshold
CENTER_DIST_THRESH  = 2.0         # meters; used with BEV criterion

# ---------- I/O helpers ----------

def jload(p: Path):
    return json.loads(p.read_text())

def jdump(obj, p: Path):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(obj, indent=2))

def _as3x3(R):
    return np.array(R, dtype=float).reshape(3,3)

def _as3x1(t):
    arr = np.array(t, dtype=float).reshape(-1)
    return arr[:3].reshape(3,1)

# ---------- Vehicle transforms (lidar -> novatel -> world) ----------

def load_lidar_to_novatel(p: Path):
    if not p.exists():
        return np.eye(3), np.zeros((3,1))
    d = jload(p)
    if "transform" in d:
        d = d["transform"]
    R = _as3x3(d["rotation"])
    T = _as3x1(d["translation"])
    return R, T

def load_novatel_to_world(p: Path):
    if not p.exists():
        return np.eye(3), np.zeros((3,1))
    d = jload(p)
    R = _as3x3(d["rotation"])
    T = _as3x1(d["translation"])
    return R, T

# ---------- Infrastructure transforms (prefer direct virtuallidar_to_world) ----------

def _load_rt_json(p: Path):
    """Load a JSON with {rotation: 3x3 or list(9), translation: list(3)}. Returns (R,T) or (None,None) if missing."""
    if not p.exists():
        return None, None
    d = jload(p)
    src = d.get("transform", d)
    if "rotation" in src and "translation" in src:
        try:
            R = _as3x3(src["rotation"])
            T = _as3x1(src["translation"])
            return R, T
        except Exception:
            pass
    return None, None

def load_infra_lidar_to_world(stem: str):
    """
    Try infrastructure-side direct virtuallidar_to_world/<id>.json
    else chain: base_to_world @ virtuallidar_to_base
    Returns (R_i2w, T_i2w). If nothing found, returns (I,0).
    """
    # Direct first
    R, T = _load_rt_json(INF_CAL_VL2W / f"{stem}.json")
    if (R is not None) and (T is not None):
        return R, T

    # Chain fallback
    R_v2b, T_v2b = _load_rt_json(INF_CAL_VL2BASE / f"{stem}.json")
    R_b2w, T_b2w = _load_rt_json(INF_CAL_BASE2W / f"{stem}.json")
    if (R_v2b is not None) and (R_b2w is not None):
        return chain(R_v2b, T_v2b, R_b2w, T_b2w)  # virtuallidar->base then base->world

    # Nothing found
    print(f"[WARN] No infra world transform for '{stem}' (looked in '{INF_CAL_VL2W}', or chain via base). Using identity.")
    return np.eye(3), np.zeros((3,1))

# ---------- Rigid composition ----------

def chain(R_ba, T_ba, R_cb, T_cb):
    """A->B then B->C gives A->C (compose rigid transforms)."""
    R = R_cb @ R_ba
    T = R_cb @ T_ba + T_cb
    return R, T

# ---------- Box geometry ----------

def box_corners_local(l, w, h):
    """
    8 corners in the box local frame (centered), order: 4 bottom then 4 top.
    Axes: l (x), w (y), h (z).
    """
    x = l / 2.0; y = w / 2.0; z = h / 2.0
    return np.array([
        [ +x, +y, -z ],
        [ +x, -y, -z ],
        [ -x, -y, -z ],
        [ -x, +y, -z ],
        [ +x, +y, +z ],
        [ +x, -y, +z ],
        [ -x, -y, +z ],
        [ -x, +y, +z ],
    ], dtype=float)

def Rz(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]], dtype=float)

# ---------- Label parsing ----------

def parse_objects_lidar(file_path: Path):
    """
    Supports:
      - list of rich dicts
      - legacy {"boxes_3d": [[x,y,z,l,w,h,yaw], ...]}
    """
    if not file_path.exists():
        return []
    data = jload(file_path)
    if isinstance(data, list):
        return data
    objs = []
    if isinstance(data, dict) and "boxes_3d" in data:
        for arr in data["boxes_3d"]:
            x,y,z,l,w,h,yaw = arr
            objs.append({
                "type": "car",
                "occluded_state": 0,
                "truncated_state": 0,
                "alpha": 0.0,
                "2d_box": {"xmin":0,"ymin":0,"xmax":0,"ymax":0},
                "3d_dimensions": {"h": float(h), "w": float(w), "l": float(l)},
                "3d_location": {"x": float(x), "y": float(y), "z": float(z)},
                "rotation": float(yaw)
            })
    return objs

def obj_lwh_xyz_yaw(obj):
    dims = obj.get("3d_dimensions", {})
    loc  = obj.get("3d_location", {})
    h = float(dims.get("h", 1.5))
    w = float(dims.get("w", 1.8))
    l = float(dims.get("l", 4.0))
    x = float(loc.get("x", 0.0))
    y = float(loc.get("y", 0.0))
    z = float(loc.get("z", 0.0))
    yaw = float(obj.get("rotation", 0.0))
    return l, w, h, x, y, z, yaw

def to_required_schema_world(obj_in, center_w, yaw_w, corners_w):
    typ = obj_in.get("type", "car")
    occluded_state  = obj_in.get("occluded_state", 0)
    truncated_state = obj_in.get("truncated_state", 0)
    alpha           = obj_in.get("alpha", 0.0)
    box2d = obj_in.get("2d_box", {"xmin":0.0,"ymin":0.0,"xmax":0.0,"ymax":0.0})
    dims = obj_in.get("3d_dimensions", {})
    h = float(dims.get("h", 1.5))
    w = float(dims.get("w", 1.8))
    l = float(dims.get("l", 4.0))
    return {
        "type": typ,
        "occluded_state": int(occluded_state),
        "truncated_state": int(truncated_state),
        "alpha": float(alpha),
        "2d_box": {
            "xmin": float(box2d.get("xmin", 0.0)),
            "ymin": float(box2d.get("ymin", 0.0)),
            "xmax": float(box2d.get("xmax", 0.0)),
            "ymax": float(box2d.get("ymax", 0.0)),
        },
        "3d_dimensions": {"h": float(h), "w": float(w), "l": float(l)},
        "3d_location": {"x": float(center_w[0]), "y": float(center_w[1]), "z": float(center_w[2])},
        "rotation": float(yaw_w),
        "world_8_points": corners_w.tolist()
    }

# ---------- Geometry helpers for IoU ----------

def _bottom_face_corners_from_world8(world8: np.ndarray):
    """
    Given 8 world corners in our ordering (0..3 bottom, 4..7 top),
    return the 4 bottom XY points in order.
    """
    if world8.shape != (8,3):
        raise ValueError("world8 must be (8,3)")
    return world8[:4, :2]  # (4,2)

def _poly_area(pts: np.ndarray) -> float:
    """Shoelace area; pts: (N,2) in order (convex rectangle here)."""
    x = pts[:,0]; y = pts[:,1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))

def _suth_hodg_clip(subject: np.ndarray, clipper: np.ndarray) -> np.ndarray:
    """
    Sutherland–Hodgman polygon clipping for convex polygons.
    subject, clipper: (N,2), (M,2). Returns (K,2) possibly empty.
    """
    def inside(p, a, b):
        # keep left side of edge a->b
        return (b[0]-a[0])*(p[1]-a[1]) - (b[1]-a[1])*(p[0]-a[0]) >= 0.0

    def intersect(p1, p2, a, b):
        # line p1->p2 with edge a->b
        x1,y1 = p1; x2,y2 = p2; x3,y3 = a; x4,y4 = b
        denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
        if abs(denom) < 1e-12:
            return p2  # nearly parallel; return p2 to keep stability
        px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / denom
        py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / denom
        return np.array([px, py], dtype=float)

    output = subject.copy()
    for i in range(len(clipper)):
        input_list = output
        if input_list.shape[0] == 0:
            break
        output = []
        A = clipper[i]
        B = clipper[(i+1) % len(clipper)]
        S = input_list[-1]
        for E in input_list:
            if inside(E, A, B):
                if not inside(S, A, B):
                    output.append(intersect(S, E, A, B))
                output.append(E)
            elif inside(S, A, B):
                output.append(intersect(S, E, A, B))
            S = E
        output = np.array(output, dtype=float)
    return output

def _bev_intersection_area(rectA: np.ndarray, rectB: np.ndarray) -> float:
    """
    rectA, rectB: (4,2) bottom-face XY corners (convex, ordered).
    Returns intersection area.
    """
    inter = _suth_hodg_clip(rectA, rectB)
    if inter.shape[0] == 0:
        return 0.0
    return _poly_area(inter)

def _height_overlap(zA: float, hA: float, zB: float, hB: float) -> float:
    A_min, A_max = zA - hA/2.0, zA + hA/2.0
    B_min, B_max = zB - hB/2.0, zB + hB/2.0
    return max(0.0, min(A_max, B_max) - max(A_min, B_min))

def oriented_iou_3d(a, b) -> tuple[float, float]:
    """
    Compute oriented 3D IoU and BEV IoU using stored world_8_points and dims.
    a, b are dicts in our output schema (already in world).
    Returns (iou3d, iou_bev).
    """
    # bottom rectangles
    A_xy = _bottom_face_corners_from_world8(np.array(a["world_8_points"], dtype=float))
    B_xy = _bottom_face_corners_from_world8(np.array(b["world_8_points"], dtype=float))

    # areas
    areaA = _poly_area(A_xy)
    areaB = _poly_area(B_xy)
    inter_area = _bev_intersection_area(A_xy, B_xy)
    union_area = max(areaA + areaB - inter_area, 1e-12)
    iou_bev = inter_area / union_area

    # vertical overlap
    zA = float(a["3d_location"]["z"]); hA = float(a["3d_dimensions"]["h"])
    zB = float(b["3d_location"]["z"]); hB = float(b["3d_dimensions"]["h"])
    inter_h = _height_overlap(zA, hA, zB, hB)
    if inter_h <= 0.0 or inter_area <= 0.0:
        return 0.0, iou_bev

    inter_vol = inter_area * inter_h
    volA = areaA * hA
    volB = areaB * hB
    union_vol = max(volA + volB - inter_vol, 1e-12)
    iou3d = inter_vol / union_vol
    return iou3d, iou_bev

def centers_dist(a, b) -> float:
    ax, ay, az = float(a["3d_location"]["x"]), float(a["3d_location"]["y"]), float(a["3d_location"]["z"])
    bx, by, bz = float(b["3d_location"]["x"]), float(b["3d_location"]["y"]), float(b["3d_location"]["z"])
    return float(np.linalg.norm([ax-bx, ay-by, az-bz]))

def same_class(a, b) -> bool:
    return str(a.get("type","")).lower() == str(b.get("type","")).lower()

# ---------- Main ----------

def main():
    if not VEH_LABEL_DIR.exists():
        print(f"ERROR: {VEH_LABEL_DIR} not found")
        return

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    files = sorted(VEH_LABEL_DIR.glob("*.json"))
    written = 0

    for f in files:
        vid = f.stem

        # --- VEHICLE SIDE ---
        veh_objs_lidar = parse_objects_lidar(f)

        # LiDAR->World via (lidar->novatel) then (novatel->world)
        R_l2n, T_l2n = load_lidar_to_novatel(VEH_CAL_L2NOV / f"{vid}.json")
        R_n2w, T_n2w = load_novatel_to_world(VEH_CAL_NOV2W / f"{vid}.json")
        R_v_l2w, T_v_l2w = chain(R_l2n, T_l2n, R_n2w, T_n2w)

        merged_world_list: list[dict] = []

        # Transform & keep all vehicle boxes
        for obj in veh_objs_lidar:
            l, w, h, x, y, z, yaw_l = obj_lwh_xyz_yaw(obj)

            center_l = np.array([x, y, z], dtype=float).reshape(3,1)
            center_w = (R_v_l2w @ center_l + T_v_l2w).reshape(3)

            R_obj_w = R_v_l2w @ Rz(yaw_l)
            yaw_w = float(np.arctan2(R_obj_w[1,0], R_obj_w[0,0]))

            corners_local = box_corners_local(l, w, h)
            corners_world = (R_obj_w @ corners_local.T).T + center_w.reshape(1,3)

            merged_world_list.append(to_required_schema_world(obj, center_w, yaw_w, corners_world))

        # --- INFRASTRUCTURE SIDE (if present) ---
        infra_label_path = INF_LABEL_DIR / f"{vid}.json"
        infra_objs_lidar = parse_objects_lidar(infra_label_path)
        if len(infra_objs_lidar) > 0:
            R_i_l2w, T_i_l2w = load_infra_lidar_to_world(vid)

            for obj in infra_objs_lidar:
                l, w, h, x, y, z, yaw_l = obj_lwh_xyz_yaw(obj)

                center_l = np.array([x, y, z], dtype=float).reshape(3,1)
                center_w = (R_i_l2w @ center_l + T_i_l2w).reshape(3)

                R_obj_w = R_i_l2w @ Rz(yaw_l)
                yaw_w = float(np.arctan2(R_obj_w[1,0], R_obj_w[0,0]))

                corners_local = box_corners_local(l, w, h)
                corners_world = (R_obj_w @ corners_local.T).T + center_w.reshape(1,3)

                cand = to_required_schema_world(obj, center_w, yaw_w, corners_world)

                # ---------- DEDUP vs already-kept (mostly vehicle first) ----------
                is_duplicate = False
                for kept in merged_world_list:
                    # If classes disagree, allow both (skip strict class check if you want)
                    if not same_class(cand, kept):
                        continue
                    iou3d, ioubev = oriented_iou_3d(cand, kept)
                    if (iou3d >= IOU3D_THRESH) or (ioubev >= IOU_BEV_THRESH and centers_dist(cand, kept) <= CENTER_DIST_THRESH):
                        # Duplicate found
                        if PREFER_ON_DUPLICATE == "infra":
                            # Replace kept with cand
                            idx = merged_world_list.index(kept)
                            merged_world_list[idx] = cand
                        # else prefer vehicle => do nothing (skip cand)
                        is_duplicate = True
                        break

                if not is_duplicate:
                    merged_world_list.append(cand)

        # --- WRITE MERGED WORLD LABELS (deduplicated) ---
        out_p = OUT_DIR / f"{vid}.json"
        jdump(merged_world_list, out_p)
        written += 1

    print(f"[done] wrote {written} WORLD-frame cooperative label files (veh + infra dedup) to: {OUT_DIR}")

if __name__ == "__main__":
    main()
