#!/usr/bin/env python3
"""
Auto-select targets from CSV by actor_label keywords, verify they are in the
current camera FOV and within range, then run the SAME per-target pipeline
you already had (3D cuboid + amodal 2D box + ROI->Open3D + optional tight refit).

Also shows a full-frame segmentation image clipped by depth <= DEPTH_CLIP_MAX_M.

IMPORTANT: All camera data AND all object poses are sampled during a SINGLE
simPause(True) interval. No additional pauses occur after that, so all 3D boxes
correspond to the same simulation time-step.
"""

import math, re
import numpy as np
import setup_path
import hercules as airsim
import cv2
import csv
from collections import defaultdict

# optional visualization
try:
    import open3d as o3d
except ImportError:
    o3d = None

# === CONFIGURATION ===
TARGETS = []  # will be built automatically from CSV + scene + FOV

CAMERA_NAME        = "front_center"
CLIENT_CLASS       = airsim.MultirotorClient
PORT               = 41451

PROJECTION_ENABLED = True

# Draw control:
# - If True: draw ONLY the corrected/tight 2D boxes (orange) on the Scene RGB window.
#            The initial amodal boxes and edge-overlays will NOT be drawn.
# - If False: draw both the amodal (initial) box/edges and the corrected/tight boxes.
DRAW_ONLY_CORRECTED_2D = True
TIGHT_BOX_COLOR_BGR    = (0,165,255)   # orange for tight (corrected) box

# NEW: Enforce dominant-color logic & occlusion filter for 2D drawing
# When True:
#   * We DO NOT draw any amodal/edge boxes.
#   * We ONLY draw the tight box around the dominant color inside the 3D box.
#   * If no ROI points fall inside the 3D box (occluded), the object is ignored entirely.
DOMINANT_COLOR_ONLY = True

# Show full-frame segmentation with small-color pruning?
SEG_FULL_PRUNE_COLORS = False  # set True to prune tiny color islands; False = show all colors


# --- 2D bbox size gating (after color selection) ---
DOMINANT_MIN_PIXELS = 120   # require this many pixels (post-clip) for the chosen color
MIN_BBOX_WIDTH      = 20    # px
MIN_BBOX_HEIGHT     = 20    # px
MIN_BBOX_AREA       = 400   # px^2

# --- per-object profiles (dimensions in meters; Z is NED +Z down)
PROFILES = {
    "human": {"L": 0.5, "W": 0.75, "H": 1.9,  "Z": -0.90},
    "car":   {"L": 4.2, "W": 1.90, "H": 1.60, "Z": -0.55},
}

# --- OPTIONAL: per-ID substring overrides (applied whenever substring appears in idname)
# Customize these numbers to your desired pickup dimensions and Z offset.
PROFILE_OVERRIDES_BY_ID_SUBSTR = {
    # example override for any id containing "pickup"
    "pickup": {"L": 5.35, "W": 2.05, "H": 2.2, "Z": -1.0, "FWD_OFF_M": -0.45},
    # add more substrings if needed...
}

SHOW_ROI_WINDOWS_GLOBAL = False  # avoid N windows when many auto targets are found

# --- Depth-clip settings ---
RANGE_MAX_M   = 40.0
DEPTH_CLIP_ENABLE     = True
DEPTH_CLIP_MAX_M      = RANGE_MAX_M   # keep in sync with detection range
SHOW_ORIGINAL_SEG_ROI = False

# --- Full-frame segmentation color filtering ---
# Any unique color in the depth-clipped full-frame segmentation with fewer pixels
# than this threshold will be removed from the display (set to black).
MIN_SEG_COLOR_PIXELS_FULL = 2000

# --- Point cloud export from clipped ROI ---
ADD_ROI_POINTS_TO_OPEN3D = True
ROI_POINT_STRIDE          = 1

# --- tight-box refit params (used for dominant-color tight box too)
REFIT_USE_DEPTH_CLIP_FOR_TIGHT_BOX = True
REFIT_MIN_PIXELS                   = 50
REFIT_SEARCH_MARGIN_PX             = 20

# --- visible-objects print controls ---
VISIBLE_EPS_METERS = 1.0
MAX_VISIBLE_PRINT  = 200

# --- mapping csv + filters ---
CSV_PATH      = "/home/sgarimella34/multi-robot-coordination/HERCULES/csv_data/ue_label_vs_name.csv"
KEYWORDS = (
    "human", "person", "pedestrian",
    "car", "truck", "sedan", "suv", "van", "bus", "vehicle",
    "sportscar", "sports car", "policecar", "pickup", "taxi"
)
MAX_OBJECTS   = 200

# Color palette for per-object box colors (BGR for cv2)
BOX_COLORS_BGR = [
    (0,255,0), (255,0,255), (0,165,255), (255,255,0), (0,255,255),
    (255,0,0), (180,105,255), (128,0,255), (211,0,148), (0,128,255)
]

# ---- Force-included actors (processed in addition to auto/FOV selection) ----
FORCE_INCLUDE_IDNAMES = [
    "BP_VehicleAI_pickup_C_UAID_6C6E07132D49788102_1328099840",
]
FORCE_INCLUDE_OBJECT_TYPE = "car"          # or None to infer
FORCE_INCLUDE_COLOR_BGR   = (0, 0, 255)    # red, reserved for this actor

# ===================== helpers =====================
def load_actor_map(csv_path):
    mapping = {}
    try:
        with open(csv_path, "r", newline="") as f:
            sniffer = csv.Sniffer()
            sample = f.read(1024)
            f.seek(0)
            has_header = False
            try:
                has_header = sniffer.has_header(sample)
            except Exception:
                pass
            reader = csv.reader(f)
            if has_header:
                next(reader, None)
            for row in reader:
                if not row or len(row) < 2:
                    continue
                actor_label = row[0].strip()
                idname = row[1].strip()
                if idname:
                    mapping[idname] = actor_label
    except FileNotFoundError:
        print(f"CSV not found: {csv_path} (continuing without labels)")
    except Exception as e:
        print(f"Error reading CSV '{csv_path}': {e} (continuing without labels)")
    return mapping

def label_has_keyword(name_or_label, keywords):
    if not name_or_label:
        return False
    s_raw = str(name_or_label)
    s_cc = re.sub(r'(?<=[a-z])(?=[A-Z0-9])', ' ', s_raw)
    s_cc = re.sub(r'(?<=[A-Z])(?=[0-9])', ' ', s_cc)
    s_low = s_cc.lower()
    s_compact = re.sub(r'[^a-z0-9]+', '', s_low)
    for kw in keywords:
        kw_low = kw.lower()
        kw_compact = re.sub(r'[^a-z0-9]+', '', kw_low)
        if kw_low in s_low: return True
        if kw_compact and kw_compact in s_compact: return True
        if ' ' in kw_low:
            parts = re.split(r'[^a-z0-9]+', kw_low)
            parts = [re.escape(p) for p in parts if p]
            if parts:
                rx = r'(?:^|[^a-z0-9])' + r'[^a-z0-9]*'.join(parts) + r'(?:[^a-z0-9]|$)'
                if re.search(rx, s_low): return True
    return False

def infer_object_type_from_label(label):
    s = (label or "").lower()
    if ("human" in s) or ("person" in s) or ("pedestrian" in s) or ("splinehuman" in s):
        return "human"
    if any(k in s for k in ("car","truck","sedan","suv","vehicle","van","bus","sportscar","policecar","pickup")):
        return "car"
    return "car"

def cam_to_point_range(pt_world, cam_pose):
    R_cam = quaternion_to_rotation_matrix(cam_pose.orientation)
    cam_p = np.array([cam_pose.position.x_val, cam_pose.position.y_val, cam_pose.position.z_val], dtype=float)
    p_cam = R_cam.T @ (pt_world - cam_p)
    return float(np.linalg.norm(p_cam)), p_cam[0]

def quaternion_to_euler(q):
    w,x,y,z = q.w_val, q.x_val, q.y_val, q.z_val
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    if norm == 0: return 0.0, 0.0, 0.0
    w,x,y,z = w/norm, x/norm, y/norm, z/norm
    sinr = 2*(w*x + y*z);  cosr = 1 - 2*(x*x + y*y)
    roll = math.atan2(sinr, cosr)
    sinp = 2*(w*y - z*x)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp)>=1 else math.asin(sinp)
    siny = 2*(w*z + x*y);  cosy = 1 - 2*(y*y + z*z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw

def quaternion_to_rotation_matrix(q):
    w,x,y,z = q.w_val, q.x_val, q.y_val, q.z_val
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    if norm == 0: return np.eye(3)
    w,x,y,z = w/norm, x/norm, y/norm, z/norm
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y - z*w),  2*(x*z + y*w)],
        [2*(x*y + z*w),   1-2*(x*x+z*z),  2*(y*z - x*w)],
        [2*(x*z - y*w),   2*(y*z + x*w),  1-2*(x*x+y*y)]
    ], dtype=float)

def print_pose(label, pose):
    if pose is None:
        print(f"{label}: <no pose>")
        return
    p,o = pose.position, pose.orientation
    r,pit,y = quaternion_to_euler(o)
    print(f"=== {label} ===")
    print(f" Position (NED): x={p.x_val:.6f}, y={p.y_val:.6f}, z={p.z_val:.6f}")
    print(f" Quaternion (w,x,y,z): ({o.w_val:.6f}, {o.x_val:.6f}, {o.y_val:.6f}, {o.z_val:.6f})")
    print(f" Euler (deg): roll={math.degrees(r):.2f}, pitch={math.degrees(pit):.2f}, yaw={math.degrees(y):.2f}\n")

def compute_intrinsics_from_horizontal_fov(hfov_deg, width, height):
    hfov = math.radians(hfov_deg)
    fx = (width/2.0) / math.tan(hfov/2.0)
    fy = fx
    cx, cy = width/2.0, height/2.0
    K = np.array([[fx, 0, cx],[0, fy, cy],[0, 0, 1]], dtype=float)
    vfov = 2 * math.degrees(math.atan((height/2.0)/fy))
    return K, vfov

def compute_bounding_box_corners_world(pose, L, W, H):
    hl, hw, hh = L/2.0, W/2.0, H/2.0
    corners_local = np.array([
        [ hl,  hw,  hh], [ hl,  hw, -hh],
        [ hl, -hw,  hh], [ hl, -hw, -hh],
        [-hl,  hw,  hh], [-hl,  hw, -hh],
        [-hl, -hw,  hh], [-hl, -hw, -hh],
    ], dtype=float)
    R = quaternion_to_rotation_matrix(pose.orientation)
    t = np.array([pose.position.x_val, pose.position.y_val, pose.position.z_val], dtype=float)
    return (R @ corners_local.T).T + t

def project_world_points_to_image(world_pts, cam_pose, P, width, height):
    R_cam = quaternion_to_rotation_matrix(cam_pose.orientation)
    cam_p = np.array([cam_pose.position.x_val, cam_pose.position.y_val, cam_pose.position.z_val], dtype=float)
    cam_pts = (R_cam.T @ (world_pts - cam_p).T).T
    pts_h = np.hstack([cam_pts, np.ones((cam_pts.shape[0], 1), dtype=float)])
    clip  = (P @ pts_h.T).T
    w_comp = clip[:, 3:4]
    ndc   = clip[:, :3] / w_comp
    u = (1.0 - (ndc[:, 0] * 0.5 + 0.5)) * width
    v = (ndc[:, 1] * 0.5 + 0.5) * height
    pts2d = np.stack([u, v], axis=1)
    depth_forward = cam_pts[:, 0]
    valid = depth_forward > 1e-6
    return pts2d, depth_forward, valid

def draw_2d_bbox_and_get_rect(pts2d, valid, w, h, img_to_draw=None, color=(0,255,0), thickness=2):
    us = pts2d[valid, 0]; vs = pts2d[valid, 1]
    if us.size == 0 or vs.size == 0:
        return None
    x0 = int(max(0, math.floor(us.min())))
    x1 = int(min(w-1, math.ceil(us.max())))
    y0 = int(max(0, math.floor(vs.min())))
    y1 = int(min(h-1, math.ceil(vs.max())))
    if img_to_draw is not None:
        cv2.rectangle(img_to_draw, (x0,y0), (x1,y1), color, thickness)
    return (x0, y0, x1, y1)

def resize_to(img, target_w, target_h, is_depth=False):
    if img.shape[1] == target_w and img.shape[0] == target_h:
        return img
    return cv2.resize(img, (target_w, target_h), interpolation=cv2.INTER_NEAREST)

def depth_roi_to_vis(depth_roi):
    d = depth_roi.copy()
    finite = np.isfinite(d) & (d > 0)
    if np.any(finite):
        lo = np.percentile(d[finite], 2.0)
        hi = np.percentile(d[finite], 98.0)
        if hi <= lo: hi = lo + 1e-3
        d = np.clip(d, lo, hi)
        d[~finite] = hi
        vis = cv2.normalize(d, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    else:
        vis = np.zeros_like(d, dtype=np.uint8)
    return cv2.applyColorMap(vis, cv2.COLORMAP_JET)

def roi_points_to_world_pointcloud_P(seg_roi_clipped, depth_roi, x0, y0,
                                     img_w, img_h, P, cam_pose, stride=1):
    H, W = depth_roi.shape
    seg_nonzero   = np.any(seg_roi_clipped != 0, axis=2)
    finite_depth  = np.isfinite(depth_roi) & (depth_roi > 0)
    mask = seg_nonzero & finite_depth
    if not np.any(mask):
        return None, None
    if stride > 1:
        sampled = np.zeros_like(mask)
        sampled[::stride, ::stride] = True
        mask = mask & sampled
        if not np.any(mask):
            return None, None
    ys, xs = np.where(mask)
    u = (x0 + xs).astype(np.float64)
    v = (y0 + ys).astype(np.float64)
    ndc_x = 1.0 - 2.0 * (u / float(img_w))
    ndc_y = 2.0 * (v / float(img_h)) - 1.0
    p01 = float(P[0,1]); p12 = float(P[1,2])
    if abs(p01) < 1e-9 or abs(p12) < 1e-9:
        return None, None
    rat_y = -ndc_x / p01
    rat_z = -ndc_y / p12
    dirs = np.stack([np.ones_like(rat_y), rat_y, rat_z], axis=1)
    norms = np.linalg.norm(dirs, axis=1, keepdims=True)
    dirs_unit = dirs / np.maximum(norms, 1e-12)
    r = depth_roi[ys, xs].astype(np.float64)[:, None]
    p_cam = dirs_unit * r
    R_cam = quaternion_to_rotation_matrix(cam_pose.orientation)
    cam_p = np.array([cam_pose.position.x_val,
                      cam_pose.position.y_val,
                      cam_pose.position.z_val], dtype=float)
    world_pts = (R_cam @ p_cam.T).T + cam_p
    colors_bgr = seg_roi_clipped[ys, xs, :]
    colors_rgb = colors_bgr[:, ::-1].astype(np.float32) / 255.0
    return world_pts.astype(np.float64), colors_rgb

def points_inside_oriented_box(world_pts, box_pose, L, W, H, eps=1e-6):
    if world_pts is None or world_pts.size == 0:
        return np.zeros((0,), dtype=bool)
    R = quaternion_to_rotation_matrix(box_pose.orientation)
    t = np.array([box_pose.position.x_val,
                  box_pose.position.y_val,
                  box_pose.position.z_val], dtype=float)
    p_local = (R.T @ (world_pts - t).T).T
    hl, hw, hh = L/2.0 + eps, W/2.0 + eps, H/2.0 + eps
    inside = (np.abs(p_local[:,0]) <= hl) & (np.abs(p_local[:,1]) <= hw) & (np.abs(p_local[:,2]) <= hh)
    return inside

def dominant_colors_in_box(world_pts, colors_rgb, box_pose, L, W, H, top_k=3):
    if world_pts is None or colors_rgb is None or world_pts.shape[0] == 0:
        return [], 0
    inside = points_inside_oriented_box(world_pts, box_pose, L, W, H)
    n_inside = int(inside.sum())
    if n_inside == 0:
        return [], 0
    cols = (np.rint(colors_rgb[inside] * 255.0)).astype(np.uint8)
    uniq, counts = np.unique(cols, axis=0, return_counts=True)
    order = np.argsort(-counts)
    uniq = uniq[order]; counts = counts[order]
    results = []
    for i in range(min(top_k, uniq.shape[0])):
        r,g,b = [int(v) for v in uniq[i]]
        frac = float(counts[i]) / float(n_inside)
        results.append(((r,g,b), int(counts[i]), frac))
    return results, n_inside

def tight_box_for_color(seg_img, depth_img, target_rgb, search_rect, use_depth=True,
                        depth_max=35.0, min_pixels=50):
    h, w = seg_img.shape[:2]
    x0, y0, x1, y1 = search_rect
    x0 = max(0, min(w-1, x0)); x1 = max(0, min(w-1, x1))
    y0 = max(0, min(h-1, y0)); y1 = max(0, min(h-1, y1))
    if x1 <= x0 or y1 <= y0:
        return None, 0
    roi_seg   = seg_img[y0:y1+1, x0:x1+1, :]
    color_bgr = np.array(target_rgb[::-1], dtype=np.uint8)
    mask_color = np.all(roi_seg == color_bgr, axis=2)
    if use_depth and depth_img is not None:
        roi_depth = depth_img[y0:y1+1, x0:x1+1]
        depth_ok  = np.isfinite(roi_depth) & (roi_depth > 0) & (roi_depth <= depth_max)
        mask = mask_color & depth_ok
    else:
        mask = mask_color
    ys, xs = np.where(mask)
    if xs.size < min_pixels:
        return None, 0
    bx0 = int(x0 + xs.min()); bx1 = int(x0 + xs.max())
    by0 = int(y0 + ys.min()); by1 = int(y0 + ys.max())
    return (bx0, by0, bx1, by1), int(xs.size)

def world_to_cam_and_pixel(pt_world, cam_pose, P, img_w, img_h):
    R_cam = quaternion_to_rotation_matrix(cam_pose.orientation)
    cam_p = np.array([cam_pose.position.x_val, cam_pose.position.y_val, cam_pose.position.z_val], dtype=float)
    p_cam = R_cam.T @ (pt_world - cam_p)
    in_front = p_cam[0] > 1e-6
    pts_h = np.array([[p_cam[0], p_cam[1], p_cam[2], 1.0]], dtype=float)
    clip  = (P @ pts_h.T).T
    ndc   = (clip[:, :3] / clip[:, 3:4])[0]
    u = (1.0 - (ndc[0] * 0.5 + 0.5)) * img_w
    v = (ndc[1] * 0.5 + 0.5) * img_h
    in_bounds = (u >= 0) and (u < img_w) and (v >= 0) and (v < img_h)
    return p_cam, float(u), float(v), bool(in_front), bool(in_bounds)

def approx_visible(pt_world, cam_pose, P, img_w, img_h, depth_img, eps=1.0):
    p_cam, u, v, in_front, in_bounds = world_to_cam_and_pixel(pt_world, cam_pose, P, img_w, img_h)
    if not (in_front and in_bounds):
        return False
    r_obj = float(np.linalg.norm(p_cam))
    if not np.isfinite(r_obj) or r_obj <= 0:
        return False
    ui, vi = int(round(u)), int(round(v))
    ui = max(0, min(img_w - 1, ui))
    vi = max(0, min(img_h - 1, vi))
    r_depth = float(depth_img[vi, ui]) if depth_img is not None else float("inf")
    if not np.isfinite(r_depth) or r_depth <= 0:
        return False
    return abs(r_depth - r_obj) <= eps

def _safe_get_pose(client, name):
    """Try newer signature with 'True', fall back if unsupported."""
    try:
        return client.simGetObjectPose(name, True)
    except TypeError:
        return client.simGetObjectPose(name)

def resolve_id_exact_or_prefix(client, idname):
    """
    Return an existing scene ID that matches exactly, OR fall back to the same
    class prefix if the UAID changed (blueprints often respawn with new suffixes).
    """
    m = client.simListSceneObjects(f"^{re.escape(idname)}$")
    if m:
        return m[0]
    prefix = idname
    if "_UAID_" in idname:
        prefix = idname.split("_UAID_")[0] + r"_UAID_"
    candidates = client.simListSceneObjects(f"^{re.escape(prefix)}.*$")
    if candidates:
        print(f"[force] exact ID not found; using '{candidates[0]}' via prefix match.")
        return candidates[0]
    print(f"[force] could not resolve '{idname}' (no exact or prefix match).")
    return None

# ---- profile override helpers --------------------------------------------------

def get_profile_for_idname(idname, obj_type):
    """Start from PROFILES[obj_type], apply any substring overrides for the id."""
    prof = PROFILES.get(obj_type, PROFILES["car"]).copy()
    lname = idname.lower()
    for substr, override in PROFILE_OVERRIDES_BY_ID_SUBSTR.items():
        if substr in lname:
            prof.update(override)
            break
    # defaults
    if "FWD_OFF_M" not in prof:
        prof["FWD_OFF_M"] = 0.0
    return prof

def pose_with_offsets(base_pose, z_off_m=0.0, fwd_off_m=0.0):
    """
    Return a new pose where the position is shifted by:
      - +z_off_m in world Z (NED +Z is down in your setup)
      - +fwd_off_m along the actor's local +X axis (forward)
    Orientation is unchanged.
    """
    R = quaternion_to_rotation_matrix(base_pose.orientation)  # 3x3
    fwd = R[:, 0]  # local +X expressed in world
    p = np.array([base_pose.position.x_val,
                  base_pose.position.y_val,
                  base_pose.position.z_val], dtype=float)
    p = p + fwd_off_m * fwd
    p[2] = p[2] + z_off_m
    return airsim.Pose(
        position_val=airsim.Vector3r(p[0], p[1], p[2]),
        orientation_val=base_pose.orientation
    )


def amodal_bbox_for_actor_with_dims(pose, cam_pose, P, img_w, img_h,
                                    L, W, H, z_off, fwd_off=0.0):
    """Same as amodal_bbox_for_actor, but with explicit dimensions/offset."""
    adj_pose = airsim.Pose(
        position_val=airsim.Vector3r(
            pose.position.x_val,
            pose.position.y_val,
            pose.position.z_val + z_off
        ),
        orientation_val=pose.orientation
    )
    corners_w = compute_bounding_box_corners_world(adj_pose, L, W, H)
    pts2d, depth_forward, valid = project_world_points_to_image(corners_w, cam_pose, P, img_w, img_h)
    if not np.any(valid):
        return None, False, corners_w, adj_pose, (L, W, H), depth_forward, valid

    u = pts2d[:, 0]; v = pts2d[:, 1]
    in_bounds = (u >= 0) & (u < img_w) & (v >= 0) & (v < img_h)
    use = valid & in_bounds
    if np.any(use):
        x0 = int(max(0, math.floor(u[use].min())))
        y0 = int(max(0, math.floor(v[use].min())))
        x1 = int(min(img_w - 1, math.ceil(u[use].max())))
        y1 = int(min(img_h - 1, math.ceil(v[use].max())))
        if x1 > x0 and y1 > y0:
            return (x0, y0, x1, y1), True, corners_w, adj_pose, (L, W, H), depth_forward, valid

    # Edge-aware fallback
    edges = [(0,1),(1,3),(3,2),(2,0),(4,5),(5,7),(7,6),(6,4),(0,4),(1,5),(2,6),(3,7)]
    clipped_points = []
    for a, b in edges:
        if not (valid[a] and valid[b]):
            continue
        seg = _clip_segment_to_rect((u[a], v[a]), (u[b], v[b]), img_w, img_h)
        if seg is not None:
            p0, p1 = seg
            clipped_points.append(p0)
            clipped_points.append(p1)

    if len(clipped_points) >= 2:
        cps = np.array(clipped_points, dtype=float)
        x0 = int(max(0, math.floor(cps[:,0].min())))
        y0 = int(max(0, math.floor(cps[:,1].min())))
        x1 = int(min(img_w - 1, math.ceil(cps[:,0].max())))
        y1 = int(min(img_h - 1, math.ceil(cps[:,1].max())))
        if x1 > x0 and y1 > y0:
            return (x0, y0, x1, y1), True, corners_w, adj_pose, (L, W, H), depth_forward, valid

    return None, True, corners_w, adj_pose, (L, W, H), depth_forward, valid

# ================================================================================

def process_target(target_cfg, client, cam_info, cam_pose, img, seg_img, depth_img, P):
    """
    Per-target pipeline. IMPORTANT: This function assumes the sim is PAUSED.

    Returns dict with drawing artifacts plus whether a tight/corrected box was drawn.
    """
    label = target_cfg["label"]
    pattern = target_cfg["ACTOR_PATTERN"]
    obj_type = target_cfg["OBJECT_TYPE"]
    box_color = target_cfg["BOX_COLOR_BGR"]
    show_roi = SHOW_ROI_WINDOWS_GLOBAL and target_cfg.get("SHOW_ROI_WINDOWS", False)

    # base profile, then apply per-target override if provided
    profile = PROFILES[obj_type].copy() if obj_type in PROFILES else PROFILES["car"].copy()
    if "PROFILE_OVERRIDE" in target_cfg and target_cfg["PROFILE_OVERRIDE"]:
        profile.update(target_cfg["PROFILE_OVERRIDE"])

    L, W, H = profile["L"], profile["W"], profile["H"]
    z_off   = profile["Z"]

    # Find actor by anchored regex
    objs = client.simListSceneObjects(pattern)
    if not objs:
        print(f"[{label}] No actor matches '{pattern}'")
        return {"found": False}
    actor = objs[0]
    print(f"[{label}] Target actor: {actor}")

    # Pose (inside paused window)
    actor_pose = _safe_get_pose(client, actor)

    # Apply Z-offset to actor centroid (NED)
    fwd_off = profile.get("FWD_OFF_M", 0.0)
    adjusted_actor_pose = pose_with_offsets(actor_pose, z_off_m=z_off, fwd_off_m=fwd_off)

    # Compute corners and project
    corners_w = compute_bounding_box_corners_world(adjusted_actor_pose, L, W, H)

    h, w = img.shape[:2]
    disp_bbox = None
    disp_img  = img
    drew_tight = False

    if PROJECTION_ENABLED and cam_pose is not None:
        pts2d, depth_forward, valid = project_world_points_to_image(corners_w, cam_pose, P, w, h)

        # compute amodal bbox (for search region), but DO NOT draw if DOMINANT_COLOR_ONLY
        u = pts2d[:, 0]; v = pts2d[:, 1]
        in_bounds = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        use = valid & in_bounds
        if np.any(use):
            x0 = int(max(0, math.floor(u[use].min())))
            y0 = int(max(0, math.floor(v[use].min())))
            x1 = int(min(w - 1, math.ceil(u[use].max())))
            y1 = int(min(h - 1, math.ceil(v[use].max())))
            if x1 > x0 and y1 > y0:
                disp_bbox = (x0, y0, x1, y1)
                if not (DOMINANT_COLOR_ONLY or DRAW_ONLY_CORRECTED_2D):
                    cv2.rectangle(disp_img, (x0, y0), (x1, y1), box_color, 2)
        if disp_bbox is None:
            # fallback: edge-clipped amodal bbox
            edges = [(0,1),(1,3),(3,2),(2,0),(4,5),(5,7),(7,6),(6,4),(0,4),(1,5),(2,6),(3,7)]
            clipped = []
            for a, b in edges:
                if not (valid[a] and valid[b]):
                    continue
                seg = _clip_segment_to_rect((u[a], v[a]), (u[b], v[b]), w, h)
                if seg is not None:
                    p0, p1 = seg
                    clipped.append(p0); clipped.append(p1)
                    if not (DOMINANT_COLOR_ONLY or DRAW_ONLY_CORRECTED_2D):
                        cv2.line(disp_img, (int(p0[0]), int(p0[1])), (int(p1[0]), int(p1[1])), box_color, 2)
            if len(clipped) >= 2:
                cps = np.array(clipped, dtype=float)
                x0 = int(max(0, math.floor(cps[:,0].min())))
                y0 = int(max(0, math.floor(cps[:,1].min())))
                x1 = int(min(w - 1, math.ceil(cps[:,0].max())))
                y1 = int(min(h - 1, math.ceil(cps[:,1].max())))
                if x1 > x0 and y1 > y0:
                    disp_bbox = (x0, y0, x1, y1)
                    if not (DOMINANT_COLOR_ONLY or DRAW_ONLY_CORRECTED_2D):
                        cv2.rectangle(disp_img, (x0, y0), (x1, y1), box_color, 2)

        # centroid crosshair only when not restricting to corrected-only to avoid clutter
        if not (DOMINANT_COLOR_ONLY or DRAW_ONLY_CORRECTED_2D):
            cx = float(np.mean(corners_w[:,0])); cy = float(np.mean(corners_w[:,1])); cz = float(np.mean(corners_w[:,2]))
            p_cam, u_c, v_c, in_front, in_bounds_c = world_to_cam_and_pixel(
                np.array([cx, cy, cz], dtype=float), cam_pose, P, w, h
            )
            if in_front and in_bounds_c:
                cv2.drawMarker(disp_img, (int(round(u_c)), int(round(v_c))), box_color,
                               markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
    else:
        print(f"[{label}] Skipping projection.")

    # Build ROI and back-project clipped points for Open3D + dominant color logic
    roi_pcd = None
    roi_world_pts = None
    roi_colors_rgb = None

    if disp_bbox is not None:
        x0, y0, x1, y1 = disp_bbox
        x0 = max(0, min(w-1, x0)); x1 = max(0, min(w-1, x1))
        y0 = max(0, min(h-1, y0)); y1 = max(0, min(h-1, y1))
        if x1 > x0 and y1 > y0:
            seg_roi   = seg_img[y0:y1+1, x0:x1+1, :]
            depth_roi = depth_img[y0:y1+1, x0:x1+1]

            if DEPTH_CLIP_ENABLE:
                valid_depth_mask = np.isfinite(depth_roi) & (depth_roi > 0) & (depth_roi <= DEPTH_CLIP_MAX_M)
                seg_roi_clipped = np.zeros_like(seg_roi)
                seg_roi_clipped[valid_depth_mask] = seg_roi[valid_depth_mask]

                if ADD_ROI_POINTS_TO_OPEN3D and o3d is not None:
                    world_pts, colors = roi_points_to_world_pointcloud_P(
                        seg_roi_clipped, depth_roi, x0, y0, w, h, P, cam_pose, stride=ROI_POINT_STRIDE
                    )
                    if world_pts is not None and world_pts.shape[0] > 0:
                        roi_world_pts = world_pts
                        roi_colors_rgb = colors
                        pcd = o3d.geometry.PointCloud()
                        pcd.points = o3d.utility.Vector3dVector(world_pts)
                        pcd.colors = o3d.utility.Vector3dVector(colors)
                        roi_pcd = pcd
                        print(f"[{label}] ROI point cloud: {world_pts.shape[0]} points (P-based).")
                    else:
                        print(f"[{label}] ROI point cloud: no valid points.")
                if show_roi:
                    depth_vis = depth_roi_to_vis(depth_roi)
                    if SHOW_ORIGINAL_SEG_ROI:
                        cv2.namedWindow(f"{label} Seg ROI (raw)", cv2.WINDOW_NORMAL)
                        cv2.imshow(f"{label} Seg ROI (raw)", seg_roi)
                    cv2.namedWindow(f"{label} Seg ROI (depth <= {DEPTH_CLIP_MAX_M:.1f} m)", cv2.WINDOW_NORMAL)
                    cv2.imshow(f"{label} Seg ROI (depth <= {DEPTH_CLIP_MAX_M:.1f} m)", seg_roi_clipped)
                    cv2.namedWindow(f"{label} Depth ROI", cv2.WINDOW_NORMAL)
                    cv2.imshow(f"{label} Depth ROI", depth_vis)
            else:
                seg_roi_clipped = seg_roi  # no depth gating if disabled

            # === Dominant color INSIDE the 3D cuboid ===
            if roi_world_pts is not None and roi_colors_rgb is not None:
                top_colors, n_inside = dominant_colors_in_box(
                    roi_world_pts, roi_colors_rgb,
                    adjusted_actor_pose, L, W, H, top_k=3
                )
                print(f"[{label}] Points inside 3D box: {n_inside}")

                # If object has no colored pixels inside the 3D box → treat as occluded and ignore entirely
                if DOMINANT_COLOR_ONLY and n_inside == 0:
                    print(f"[{label}] Occluded (no colored ROI points inside cuboid). Ignoring object.")
                    return {"found": False}

                if n_inside > 0 and len(top_colors) > 0:
                    best_rgb, best_count, best_frac = top_colors[0]

                    # Search region: amodal box ± margin
                    ax0, ay0, ax1, ay1 = disp_bbox
                    margin = REFIT_SEARCH_MARGIN_PX
                    search_rect = (
                        max(0, ax0 - margin),
                        max(0, ay0 - margin),
                        min(w-1, ax1 + margin),
                        min(h-1, ay1 + margin),
                    )
                    tight_box, pix_count = tight_box_for_color(
                        seg_img,
                        depth_img,
                        best_rgb,
                        search_rect=search_rect,
                        use_depth=REFIT_USE_DEPTH_CLIP_FOR_TIGHT_BOX and DEPTH_CLIP_ENABLE,
                        depth_max=DEPTH_CLIP_MAX_M,
                        min_pixels=REFIT_MIN_PIXELS
                    )
                    if tight_box is not None:
                        tx0, ty0, tx1, ty1 = tight_box

                        # --- Final small-box gating on the corrected box ---
                        MIN_TIGHT_BOX_AREA_PX = MIN_BBOX_AREA
                        MIN_TIGHT_BOX_MIN_SIDE = min(MIN_BBOX_WIDTH, MIN_BBOX_HEIGHT)
                        w_box = tx1 - tx0
                        h_box = ty1 - ty0
                        if (w_box * h_box) < MIN_TIGHT_BOX_AREA_PX or min(w_box, h_box) < MIN_TIGHT_BOX_MIN_SIDE:
                            print(f"[{label}] Tight box rejected: too small ({w_box}x{h_box}px).")
                        else:
                            cv2.rectangle(disp_img, (tx0,ty0), (tx1,ty1), TIGHT_BOX_COLOR_BGR, 2)
                            cv2.putText(disp_img, f"{label}: dominant color box", (tx0, max(0,ty0-6)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, TIGHT_BOX_COLOR_BGR, 1)
                            print(f"[{label}] Dominant-color tight 2D box: ({tx0},{ty0})-({tx1},{ty1})")
                            drew_tight = True
                    else:
                        print(f"[{label}] Dominant color present but tight 2D box not found (min_pixels={REFIT_MIN_PIXELS}).")
                else:
                    # No colors tallied → either empty ROI or all filtered out by depth/pruning
                    if DOMINANT_COLOR_ONLY:
                        print(f"[{label}] No dominant color candidates. Ignoring object.")
                        return {"found": False}
            else:
                if DOMINANT_COLOR_ONLY:
                    print(f"[{label}] No ROI 3D points/colors to evaluate. Ignoring object.")
                    return {"found": False}
        else:
            print(f"[{label}] Amodal bbox collapsed after clipping; no ROI to process.")
            if DOMINANT_COLOR_ONLY:
                return {"found": False}

    else:
        # No amodal bbox (e.g., fully off-screen) — ignore if dominant-only is enabled
        if DOMINANT_COLOR_ONLY:
            return {"found": False}

    return {
        "found": True,
        "label": label,
        "actor_name": actor,
        "actor_pose": actor_pose,
        "adjusted_pose": adjusted_actor_pose,
        "L": L, "W": W, "H": H,
        "corners_w": corners_w,
        "roi_pcd": roi_pcd,
        "box_color": box_color,
        "drew_tight": drew_tight
    }

def obb_from_points(world_pts):
    import numpy as _np
    import open3d as _o3d
    if world_pts is None or len(world_pts) < 10:
        return None
    pcd = _o3d.geometry.PointCloud()
    pcd.points = _o3d.utility.Vector3dVector(world_pts.astype(_np.float64))
    obb = pcd.get_oriented_bounding_box()
    c = _np.asarray(obb.center, dtype=float)
    R = _np.asarray(obb.R, dtype=float)
    extents = _np.asarray(obb.extent, dtype=float)
    return c, R, extents

# ------------------ build targets from CSV + scene + FOV ------------------
def amodal_bbox_for_actor(pose, label, cam_pose, P, img_w, img_h):
    obj_type = infer_object_type_from_label(label)
    prof = PROFILES.get(obj_type, PROFILES["car"])
    L, W, H, z_off = prof["L"], prof["W"], prof["H"], prof["Z"]
    adj_pose = airsim.Pose(
        position_val=airsim.Vector3r(
            pose.position.x_val,
            pose.position.y_val,
            pose.position.z_val + z_off
        ),
        orientation_val=pose.orientation
    )
    corners_w = compute_bounding_box_corners_world(adj_pose, L, W, H)
    pts2d, depth_forward, valid = project_world_points_to_image(
        corners_w, cam_pose, P, img_w, img_h
    )
    if not np.any(valid):
        return None, False, corners_w, adj_pose, (L, W, H), depth_forward, valid
    u = pts2d[:, 0]; v = pts2d[:, 1]
    in_bounds = (u >= 0) & (u < img_w) & (v >= 0) & (v < img_h)
    use = valid & in_bounds
    if np.any(use):
        x0 = int(max(0, math.floor(u[use].min())))
        y0 = int(max(0, math.floor(v[use].min())))
        x1 = int(min(img_w - 1, math.ceil(u[use].max())))
        y1 = int(min(img_h - 1, math.ceil(v[use].max())))
        if x1 > x0 and y1 > y0:
            return (x0, y0, x1, y1), True, corners_w, adj_pose, (L, W, H), depth_forward, valid
    # Edge-aware fallback
    edges = [(0,1),(1,3),(3,2),(2,0),(4,5),(5,7),(7,6),(6,4),(0,4),(1,5),(2,6),(3,7)]
    clipped_points = []
    for a, b in edges:
        if not (valid[a] and valid[b]):
            continue
        seg = _clip_segment_to_rect((u[a], v[a]), (u[b], v[b]), img_w, img_h)
        if seg is not None:
            p0, p1 = seg
            clipped_points.append(p0)
            clipped_points.append(p1)
    if len(clipped_points) >= 2:
        cps = np.array(clipped_points, dtype=float)
        x0 = int(max(0, math.floor(cps[:,0].min())))
        y0 = int(max(0, math.floor(cps[:,1].min())))
        x1 = int(min(img_w - 1, math.ceil(cps[:,0].max())))
        y1 = int(min(img_h - 1, math.ceil(cps[:,1].max())))
        if x1 > x0 and y1 > y0:
            return (x0, y0, x1, y1), True, corners_w, adj_pose, (L, W, H), depth_forward, valid
    return None, True, corners_w, adj_pose, (L, W, H), depth_forward, valid

def build_targets_from_csv_scene(client, id_to_label, cam_pose, P, img_w, img_h):
    try:
        scene_ids = list(client.simListSceneObjects(".*"))
    except Exception as e:
        print("simListSceneObjects failed:", e)
        scene_ids = []

    dropped_keyword = 0
    dropped_geom    = 0
    dropped_depth   = 0

    candidates = []
    for idname in scene_ids:
        label = id_to_label.get(idname, "")
        if not (label_has_keyword(label, KEYWORDS) or label_has_keyword(idname, KEYWORDS)):
            dropped_keyword += 1
            continue

        text_for_type = label if label else idname
        obj_type = infer_object_type_from_label(text_for_type)

        # Pose (PAUSED)
        pose = _safe_get_pose(client, idname)
        if pose is None:
            continue

        # --- APPLY PROFILE OVERRIDES BY ID SUBSTRING HERE ---
        prof = get_profile_for_idname(idname, obj_type)

        bbox, has_any_valid, corners_w, adj_pose, (L, W, H), depth_fwd, valid = \
        amodal_bbox_for_actor_with_dims(
            pose, cam_pose, P, img_w, img_h,
            prof["L"], prof["W"], prof["H"], prof["Z"], prof.get("FWD_OFF_M", 0.0)
        )
        if not has_any_valid or bbox is None:
            dropped_geom += 1
            continue

        x0, y0, x1, y1 = bbox
        if (x1 - x0) * (y1 - y0) < 9:  # < 3x3 px
            dropped_geom += 1
            continue

        front_depths = depth_fwd[valid]
        if not np.any(front_depths <= DEPTH_CLIP_MAX_M + 1e-6):
            dropped_depth += 1
            continue

        near = float(np.min(front_depths))
        pretty_label = label if label else idname
        candidates.append((near, idname, pretty_label, obj_type))

    candidates.sort(key=lambda t: t[0])
    if len(candidates) > MAX_OBJECTS:
        candidates = candidates[:MAX_OBJECTS]

    print(
        f"Auto-selected {len(candidates)} target(s) by corner-in-FOV & depth≤{DEPTH_CLIP_MAX_M:.1f} m."
    )
    print(
        f"  Rejections → keyword/unknown:{dropped_keyword}, offscreen/behind:{dropped_geom}, depth>{DEPTH_CLIP_MAX_M:.1f}m:{dropped_depth}"
    )

    targets = []
    for i, (near, idname, label, obj_type) in enumerate(candidates, 1):
        color_bgr = BOX_COLORS_BGR[(i - 1) % len(BOX_COLORS_BGR)]
        pattern = f"^{re.escape(idname)}$"  # exact match

        # include any override we’d apply at runtime so the per-target pipeline uses it
        prof_override = get_profile_for_idname(idname, obj_type)

        targets.append({
            "label": label if label else idname,
            "ACTOR_PATTERN": pattern,
            "OBJECT_TYPE": obj_type,
            "BOX_COLOR_BGR": color_bgr,
            "SHOW_ROI_WINDOWS": False,
            "ENABLE_TIGHT_REFIT": True,
            "PROFILE_OVERRIDE": prof_override,   # <--- NEW
        })
    return targets

def _clip_segment_to_rect(p0, p1, w, h):
    x0, y0 = float(p0[0]), float(p0[1])
    x1, y1 = float(p1[0]), float(p1[1])
    dx, dy = x1 - x0, y1 - y0
    t0, t1 = 0.0, 1.0
    rect = [( -dx, x0 - 0.0      ),
            (  dx, (w - 1) - x0  ),
            ( -dy, y0 - 0.0      ),
            (  dy, (h - 1) - y0  )]
    for p, q in rect:
        if abs(p) < 1e-12:
            if q < 0:
                return None
        else:
            t = q / p
            if p < 0:
                if t > t0: t0 = t
            else:
                if t < t1: t1 = t
            if t0 > t1:
                return None
    cx0, cy0 = x0 + t0 * dx, y0 + t0 * dy
    cx1, cy1 = x0 + t1 * dx, y1 + t1 * dy
    return (cx0, cy0), (cx1, cy1)

# --- full-frame segmentation color pruning (after depth-clip) ---
def prune_small_seg_colors(seg_img_bgr, min_pixels):
    """Return a copy where any unique BGR color present with < min_pixels is removed (set to 0)."""
    out = np.zeros_like(seg_img_bgr)
    flat = seg_img_bgr.reshape(-1, 3)
    nz = np.any(flat != 0, axis=1)
    if not np.any(nz):
        return out
    colors, counts = np.unique(flat[nz], axis=0, return_counts=True)
    keep = colors[counts >= int(min_pixels)]
    if keep.size == 0:
        return out
    # paint back only the kept colors
    for c in keep:
        m = (seg_img_bgr[:,:,0] == c[0]) & (seg_img_bgr[:,:,1] == c[1]) & (seg_img_bgr[:,:,2] == c[2])
        out[m] = c
    return out

# ===================== main =====================
def main():
    np.set_printoptions(precision=4, suppress=True)
    client = CLIENT_CLASS(port=PORT)
    client.confirmConnection()
    print("Connected!\n")

    # Zero lens distortion if any
    dparams = client.simGetDistortionParams(CAMERA_NAME)
    print("Distortion params:", dparams)
    if any(abs(d)>1e-9 for d in dparams):
        print(" Zeroing distortion.")
        client.simSetDistortionParams(CAMERA_NAME, {"K1":0.0, "K2":0.0, "K3":0.0, "P1":0.0, "P2":0.0})
    else:
        print(" No distortion active.")
    print()

    # ===================== SINGLE PAUSE WINDOW =====================
    client.simPause(True)
    try:
        # (1) Camera info and synchronized image pack
        cam_info = client.simGetCameraInfo(CAMERA_NAME)
        cam_pose = cam_info.pose if cam_info else None

        reqs = [
            airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.Scene,         False, True),
            airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.Segmentation,  False, True),
            airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.DepthPerspective, True,  False),
        ]
        scene_resp, seg_resp, depth_resp = client.simGetImages(reqs)

        img = cv2.imdecode(np.frombuffer(scene_resp.image_data_uint8, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            print("Failed to decode Scene image"); return
        h, w = img.shape[:2]

        seg_img = cv2.imdecode(np.frombuffer(seg_resp.image_data_uint8, np.uint8), cv2.IMREAD_COLOR)
        if seg_img is None:
            print("Failed to decode Segmentation image")
            seg_img = np.zeros((h, w, 3), np.uint8)

        if depth_resp.height == 0 or depth_resp.width == 0:
            print("DepthPerspective invalid size; creating zeros.")
            depth_img = np.zeros((h, w), np.float32)
        else:
            depth_flat = np.array(depth_resp.image_data_float, dtype=np.float32)
            depth_img  = depth_flat.reshape(depth_resp.height, depth_resp.width)

        if seg_img.shape[:2] != (h, w):
            seg_img = resize_to(seg_img, w, h, is_depth=False)
        if depth_img.shape[:2] != (h, w):
            depth_img = resize_to(depth_img, w, h, is_depth=True)

        # Show full-frame depth-clipped segmentation image
        if DEPTH_CLIP_ENABLE:
            valid_depth_mask = np.isfinite(depth_img) & (depth_img > 0) & (depth_img <= DEPTH_CLIP_MAX_M)
            seg_full_clipped = np.zeros_like(seg_img)
            seg_full_clipped[valid_depth_mask] = seg_img[valid_depth_mask]

            if SEG_FULL_PRUNE_COLORS:
                seg_full_display = prune_small_seg_colors(seg_full_clipped, MIN_SEG_COLOR_PIXELS_FULL)
                nz = int(np.count_nonzero(np.any(seg_full_display != 0, axis=2)))
                print(f"Segmentation (depth <= {DEPTH_CLIP_MAX_M:.1f} m, ≥{MIN_SEG_COLOR_PIXELS_FULL}px): nonzero pixels = {nz}")
                win_title = f"Segmentation (depth <= {DEPTH_CLIP_MAX_M:.1f} m, ≥{MIN_SEG_COLOR_PIXELS_FULL}px)"
            else:
                seg_full_display = seg_full_clipped
                nz = int(np.count_nonzero(np.any(seg_full_display != 0, axis=2)))
                print(f"Segmentation (depth <= {DEPTH_CLIP_MAX_M:.1f} m): nonzero pixels = {nz}")
                win_title = f"Segmentation (depth <= {DEPTH_CLIP_MAX_M:.1f} m)"

            cv2.namedWindow(win_title, cv2.WINDOW_NORMAL)
            cv2.imshow(win_title, seg_full_display)

        # (2) Intrinsics & projection (frozen)
        K, vfov = compute_intrinsics_from_horizontal_fov(cam_info.fov, w, h)
        print(f"Resolution: {w}×{h}, HFOV: {cam_info.fov:.4f}°, VFOV: {vfov:.4f}°")
        print("K =\n", K, "\n")
        P = np.array(cam_info.proj_mat.matrix, dtype=np.float64).reshape((4,4))
        print("AirSim projection matrix P=\n", P, "\n")

        # (3) Build TARGETS automatically (PAUSED → all poses consistent)
        id_to_label = load_actor_map(CSV_PATH)
        print(f"CSV loaded from '{CSV_PATH}' with {len(id_to_label)} mappings.")
        if cam_pose is None:
            print("Camera pose unavailable; cannot select targets.")
            return

        global TARGETS
        TARGETS = build_targets_from_csv_scene(client, id_to_label, cam_pose, P, w, h)

        # (4) Force-include the requested ID(s), resolving UAID drift while PAUSED
        for idname_req in FORCE_INCLUDE_IDNAMES:
            resolved = resolve_id_exact_or_prefix(client, idname_req)
            if not resolved:
                continue
            # skip if already in targets
            already = any(re.fullmatch(t["ACTOR_PATTERN"].strip("^$"), resolved) for t in TARGETS)
            if already:
                print(f"[force] '{resolved}' already in TARGETS from FOV pass.")
                continue
            obj_type = FORCE_INCLUDE_OBJECT_TYPE or infer_object_type_from_label(
                id_to_label.get(resolved, resolved)
            )
            color_bgr = FORCE_INCLUDE_COLOR_BGR or BOX_COLORS_BGR[len(TARGETS) % len(BOX_COLORS_BGR)]
            # include per-id override for the forced target
            prof_override = get_profile_for_idname(resolved, obj_type)
            TARGETS.append({
                "label": id_to_label.get(resolved, resolved),
                "ACTOR_PATTERN": f"^{re.escape(resolved)}$",
                "OBJECT_TYPE": obj_type,
                "BOX_COLOR_BGR": color_bgr,
                "SHOW_ROI_WINDOWS": False,
                "ENABLE_TIGHT_REFIT": True,
                "PROFILE_OVERRIDE": prof_override,   # <--- NEW
            })
            print(f"[force] added '{resolved}' as OBJECT_TYPE='{obj_type}' with color BGR{color_bgr}.")

        if not TARGETS:
            print("No auto-selected targets — nothing to draw.")
            return

        # (5) Process each target while PAUSED — all poses/images are from the same step
        disp = img.copy()
        results = []
        for i, tgt in enumerate(TARGETS):
            print(f"\n--- Processing {tgt['label']} ---")
            res = process_target(tgt, client, cam_info, cam_pose, disp, seg_img, depth_img, P)
            results.append(res)
            if res.get("found", False):
                print_pose(f"{tgt['label']} Actor ({res['actor_name']})", res["actor_pose"])
                print_pose(f"{tgt['label']} Actor+Zoffset", res["adjusted_pose"])
                print(f"[{tgt['label']}] Box L×W×H = {res['L']}×{res['W']}×{res['H']} m")
                for j, c in enumerate(res["corners_w"]):
                    print(f"  [{j}] x={c[0]:.4f}, y={c[1]:.4f}, z={c[2]:.4f}")

    finally:
        # ===================== END SINGLE PAUSE WINDOW =====================
        client.simPause(False)

    # (6) 2D annotation & legend (after unpause, using frozen results)
    cv2.putText(disp, f"Camera: {CAMERA_NAME}", (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0), 2)
    y_off = 50
    legend_lines = []
    for res in results:
        if not res.get("found", False):
            continue
        label = res["label"]; bgr = res["box_color"]
        cv2.rectangle(disp, (10, y_off-12), (26, y_off+2), bgr, thickness=-1)
        suffix = ""
        if (DRAW_ONLY_CORRECTED_2D or DOMINANT_COLOR_ONLY) and not res.get("drew_tight", False):
            suffix = " (no dominant-color box)"
        cv2.putText(disp, f"{label}: {res['actor_name']}{suffix}", (34, y_off),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)
        y_off += 22
        legend_lines.append(f"  color BGR{bgr} -> {res['actor_name']}")
    if legend_lines:
        print("\nLegend (3D/2D box color -> IDname):")
        for line in legend_lines:
            print(line)

    title = "Projected 3D Bounding Box (dominant color only)" if DOMINANT_COLOR_ONLY else \
            ("Projected 3D Bounding Box (corrected only)" if DRAW_ONLY_CORRECTED_2D else
             "Projected 3D Bounding Box (auto from CSV+FOV)")
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.imshow(title, disp)
    print("Press any key to exit (after Open3D closes if opened).")
    cv2.waitKey(1)

    # (7) Open3D viz using the same frozen corners/pcds captured while PAUSED
    if o3d:
        try:
            geoms = []

            # camera frame (optional)
            Tc = np.eye(4)
            Tc[:3, :3] = quaternion_to_rotation_matrix(cam_pose.orientation)
            Tc[:3, 3]  = [cam_pose.position.x_val, cam_pose.position.y_val, cam_pose.position.z_val]
            fc = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
            fc.transform(Tc)
            geoms.append(fc)

            # Add ROI point clouds FIRST (only for kept objects)
            for res in results:
                if res.get("found", False) and res["roi_pcd"] is not None:
                    geoms.append(res["roi_pcd"])

            # Add cuboid line sets LAST so they render on top
            edges = [[0,1],[1,3],[3,2],[2,0],[4,5],[5,7],[7,6],[6,4],[0,4],[1,5],[2,6],[3,7]]
            for res in results:
                if not res.get("found", False):
                    continue
                bgr = res["box_color"]
                rgb = [bgr[2]/255.0, bgr[1]/255.0, bgr[0]/255.0]
                ls = o3d.geometry.LineSet(
                    points=o3d.utility.Vector3dVector(res["corners_w"]),
                    lines=o3d.utility.Vector2iVector(edges)
                )
                ls.colors = o3d.utility.Vector3dVector([rgb] * len(edges))
                geoms.append(ls)

            if len(geoms) > 0:
                print("Showing 3D viz in Open3D (boxes are drawn on top of points).")
                vis = o3d.visualization.Visualizer()
                vis.create_window(window_name="Open3D: ROI + 3D Boxes")
                for g in geoms:
                    vis.add_geometry(g)
                opt = vis.get_render_option()
                if hasattr(opt, "point_size"):
                    opt.point_size = 3.0
                if hasattr(opt, "line_width"):
                    opt.line_width = 3.0
                vis.run()
                vis.destroy_window()
            else:
                print("Open3D: nothing to show.")
        except Exception as e:
            print("Open3D error:", e)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
