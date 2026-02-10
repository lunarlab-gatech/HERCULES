#!/usr/bin/env python3
"""
Minimal script to:
  1) Find an object/actor by name/regex in the world.
  2) Print actor & camera poses (NED).
  3) Define a box (L,W,H) in actor frame (with Z-offset in NED), transform to world.
  4) Project 3D corners using AirSim's 4x4 projection matrix; draw amodal 2D box.
  5) ALSO show Segmentation ROI + Depth ROI inside that 2D box in separate windows.
  6) (NEW) Optionally depth-clip the segmentation ROI to keep only pixels with depth ≤ threshold.
  7) (NEW) Back-project depth-clipped seg ROI to 3D and render those points in Open3D with seg colors.
  8) (NEW) Identify which seg colors lie most inside the 3D cuboid and print the dominant color(s).
  9) (NEW) Refit the 2D bbox tightly around the dominant color in the segmentation image.
 10) (NEW) Print all scene objects and which are (approximately) visible in the current camera view.
"""

import math
import numpy as np
import setup_path                    # ensure hercules is on PYTHONPATH
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
# ACTOR_PATTERN      = "SkeletalMeshActor_UAID.*"
# ACTOR_PATTERN      = "StaticMeshActor_UAID_E08F4CF5208AA07502_2022041209.*"
# ACTOR_PATTERN = "StaticMeshActor_UAID_E08F4CF5208AA07502_2022041209"  # Sportscar_3
ACTOR_PATTERN = "BP_VehicleAI_pickup_C_UAID_6C6E07132D49788102_1328099840"  # BP_VehicleAI_pickup4

CAMERA_NAME        = "front_center"
CLIENT_CLASS       = airsim.MultirotorClient
PORT               = 41451

PROJECTION_ENABLED = True
DRAW_2D_BBOX       = True

# --- choose object profile: "human" or "car"
OBJECT_TYPE = "car"  # change to "human" for human-sized box

# --- per-object profiles (dimensions in meters; Z is NED +Z down)
PROFILES = {
    "human": {"L": 0.5, "W": 0.75, "H": 1.9,  "Z": -0.90},
    "car":   {"L": 4.2, "W": 1.90, "H": 1.60, "Z": -0.55},
}
if OBJECT_TYPE not in PROFILES:
    raise ValueError(f"Unknown OBJECT_TYPE '{OBJECT_TYPE}'. Choose from {list(PROFILES.keys())}.")

BOX_LENGTH    = PROFILES[OBJECT_TYPE]["L"]
BOX_WIDTH     = PROFILES[OBJECT_TYPE]["W"]
BOX_HEIGHT    = PROFILES[OBJECT_TYPE]["H"]
Z_OFFSET_NED  = PROFILES[OBJECT_TYPE]["Z"]

SHOW_ROI_WINDOWS   = True

# --- Depth-clip settings ---
DEPTH_CLIP_ENABLE     = True     # if True, mask seg ROI by depth threshold
DEPTH_CLIP_MAX_M      = 40.0     # keep pixels with depth <= this (meters)
SHOW_ORIGINAL_SEG_ROI = True     # also show the raw (unclipped) seg ROI for comparison

# --- Point cloud export from clipped ROI ---
ADD_ROI_POINTS_TO_OPEN3D = True  # toggle adding ROI points into Open3D scene
ROI_POINT_STRIDE          = 1    # stride ≥1; increase to subsample points (2,3,...)

# --- NEW: tight-box refit params ---
REFIT_USE_DEPTH_CLIP_FOR_TIGHT_BOX = True   # use same depth cutoff when finding tight box
REFIT_MIN_PIXELS                   = 50     # require at least this many pixels
REFIT_SEARCH_MARGIN_PX             = 20     # expand search region beyond the amodal box

# --- NEW: visible-objects print controls ---
VISIBLE_EPS_METERS = 1.0          # allowed mismatch between range-to-object and depth map
MAX_VISIBLE_PRINT  = 200          # cap how many names we print


# --- NEW CONFIG: mapping csv + filters ---
CSV_PATH      = "/home/sgarimella34/multi-robot-coordination/HERCULES/csv_data/ue_label_vs_name.csv"  # two columns: actor_label,IDname (header optional)
KEYWORDS      = ("human", "car", "truck", "sedan", "suv", "vehicle")
RANGE_MAX_M   = 40.0

# =====================
def load_actor_map(csv_path):
    """
    Returns dict: idname -> actor_label
    CSV may have header. Expected columns:
      col0=actor_label (human-readable), col1=IDname (AirSim get_name)
    """
    mapping = {}
    try:
        with open(csv_path, "r", newline="") as f:
            sniffer = csv.Sniffer()
            sample = f.read(1024)
            f.seek(0)
            has_header = sniffer.has_header(sample)
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

def name_has_keyword(idname, keywords):
    s = idname.lower()
    return any(k.lower() in s for k in keywords)

def cam_to_point_range(pt_world, cam_pose):
    R_cam = quaternion_to_rotation_matrix(cam_pose.orientation)
    cam_p = np.array([cam_pose.position.x_val, cam_pose.position.y_val, cam_pose.position.z_val], dtype=float)
    p_cam = R_cam.T @ (pt_world - cam_p)
    return float(np.linalg.norm(p_cam)), p_cam[0]  # Euclidean range, forward-x

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
    cam_pts = (R_cam.T @ (world_pts - cam_p).T).T  # Nx3

    pts_h = np.hstack([cam_pts, np.ones((cam_pts.shape[0], 1), dtype=float)])  # Nx4
    clip  = (P @ pts_h.T).T
    w_comp = clip[:, 3:4]
    ndc   = clip[:, :3] / w_comp

    u = (1.0 - (ndc[:, 0] * 0.5 + 0.5)) * width
    v = (ndc[:, 1] * 0.5 + 0.5) * height

    pts2d = np.stack([u, v], axis=1)
    depth_forward = cam_pts[:, 0]  # X-forward in camera frame
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
    """Resize to (target_h, target_w). Use NEAREST for depth/labels."""
    if img.shape[1] == target_w and img.shape[0] == target_h:
        return img
    return cv2.resize(img, (target_w, target_h), interpolation=cv2.INTER_NEAREST)

def depth_roi_to_vis(depth_roi):
    """8-bit colormap visualization from float meters; robust to outliers."""
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

# ---------- Back-project ROI pixels to 3D using AirSim P ----------
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
    u = (x0 + xs).astype(np.float64)  # full-image coords
    v = (y0 + ys).astype(np.float64)

    # pixel -> NDC, consistent with forward projection
    ndc_x = 1.0 - 2.0 * (u / float(img_w))
    ndc_y = 2.0 * (v / float(img_h)) - 1.0

    p01 = float(P[0,1])
    p12 = float(P[1,2])
    if abs(p01) < 1e-9 or abs(p12) < 1e-9:
        return None, None

    # From P: Y/X = -ndc_x / p01,  Z/X = -ndc_y / p12
    rat_y = -ndc_x / p01
    rat_z = -ndc_y / p12

    # Ray direction in AirSim cam coords (X-forward, Y-right, Z-down)
    dirs = np.stack([np.ones_like(rat_y), rat_y, rat_z], axis=1)  # (N,3)
    norms = np.linalg.norm(dirs, axis=1, keepdims=True)
    dirs_unit = dirs / np.maximum(norms, 1e-12)

    # DepthPerspective = Euclidean range along the viewing ray
    r = depth_roi[ys, xs].astype(np.float64)[:, None]
    p_cam = dirs_unit * r  # (N,3)

    # Lift to world
    R_cam = quaternion_to_rotation_matrix(cam_pose.orientation)
    cam_p = np.array([cam_pose.position.x_val,
                      cam_pose.position.y_val,
                      cam_pose.position.z_val], dtype=float)
    world_pts = (R_cam @ p_cam.T).T + cam_p

    # Colors: BGR -> RGB in [0,1]
    colors_bgr = seg_roi_clipped[ys, xs, :]
    colors_rgb = colors_bgr[:, ::-1].astype(np.float32) / 255.0
    return world_pts.astype(np.float64), colors_rgb

# --- inside-box test & color tally ---
def points_inside_oriented_box(world_pts, box_pose, L, W, H, eps=1e-6):
    if world_pts is None or world_pts.size == 0:
        return np.zeros((0,), dtype=bool)
    R = quaternion_to_rotation_matrix(box_pose.orientation)  # local->world
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
    cols = (np.rint(colors_rgb[inside] * 255.0)).astype(np.uint8)  # Nx3 uint8 RGB
    uniq, counts = np.unique(cols, axis=0, return_counts=True)
    order = np.argsort(-counts)
    uniq = uniq[order]; counts = counts[order]
    results = []
    for i in range(min(top_k, uniq.shape[0])):
        r,g,b = [int(v) for v in uniq[i]]
        frac = float(counts[i]) / float(n_inside)
        results.append(((r,g,b), int(counts[i]), frac))
    return results, n_inside

# --- NEW: tight box around a target RGB color, optionally depth-clipped, in a search rect
def tight_box_for_color(seg_img, depth_img, target_rgb, search_rect, use_depth=True,
                        depth_max=35.0, min_pixels=50):
    """
    seg_img:   full segmentation image (H,W,3) BGR
    depth_img: full depth (H,W) float32
    target_rgb: (R,G,B) tuple
    search_rect: (x0,y0,x1,y1) region to search (inclusive)
    Returns (bbox, count) where bbox=(x0,y0,x1,y1) in full image, or (None,0) if not found.
    """
    h, w = seg_img.shape[:2]
    x0, y0, x1, y1 = search_rect
    x0 = max(0, min(w-1, x0)); x1 = max(0, min(w-1, x1))
    y0 = max(0, min(h-1, y0)); y1 = max(0, min(h-1, y1))
    if x1 <= x0 or y1 <= y0:
        return None, 0

    roi_seg   = seg_img[y0:y1+1, x0:x1+1, :]
    color_bgr = np.array(target_rgb[::-1], dtype=np.uint8)  # RGB -> BGR
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

    bx0 = int(x0 + xs.min())
    bx1 = int(x0 + xs.max())
    by0 = int(y0 + ys.min())
    by1 = int(y0 + ys.max())
    return (bx0, by0, bx1, by1), int(xs.size)

# --- NEW HELPERS: approximate visibility test using DepthPerspective ---
def world_to_cam_and_pixel(pt_world, cam_pose, P, img_w, img_h):
    """Return (p_cam, u, v, in_front, in_bounds)."""
    R_cam = quaternion_to_rotation_matrix(cam_pose.orientation)
    cam_p = np.array([cam_pose.position.x_val, cam_pose.position.y_val, cam_pose.position.z_val], dtype=float)
    p_cam = R_cam.T @ (pt_world - cam_p)

    # behind camera?
    in_front = p_cam[0] > 1e-6

    # project using same NDC math as elsewhere
    pts_h = np.array([[p_cam[0], p_cam[1], p_cam[2], 1.0]], dtype=float)  # 1x4
    clip  = (P @ pts_h.T).T
    ndc   = (clip[:, :3] / clip[:, 3:4])[0]
    u = (1.0 - (ndc[0] * 0.5 + 0.5)) * img_w
    v = (ndc[1] * 0.5 + 0.5) * img_h
    in_bounds = (u >= 0) and (u < img_w) and (v >= 0) and (v < img_h)
    return p_cam, float(u), float(v), bool(in_front), bool(in_bounds)

def approx_visible(pt_world, cam_pose, P, img_w, img_h, depth_img, eps=1.0):
    """
    A lightweight test: project the object's world position.
    If it's in front, on image, and the depth map near that pixel agrees with the range,
    we treat it as visible (not fully occluded).
    """
    p_cam, u, v, in_front, in_bounds = world_to_cam_and_pixel(pt_world, cam_pose, P, img_w, img_h)
    if not (in_front and in_bounds):
        return False

    # Euclidean range to object along viewing ray
    r_obj = float(np.linalg.norm(p_cam))
    if not np.isfinite(r_obj) or r_obj <= 0:
        return False

    # sample depth map (nearest pixel)
    ui, vi = int(round(u)), int(round(v))
    ui = max(0, min(img_w - 1, ui))
    vi = max(0, min(img_h - 1, vi))
    r_depth = float(depth_img[vi, ui]) if depth_img is not None else float("inf")
    if not np.isfinite(r_depth) or r_depth <= 0:
        return False

    return abs(r_depth - r_obj) <= eps

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

    # Find target actor
    objs = client.simListSceneObjects(ACTOR_PATTERN)
    if not objs:
        print(f"No actor matches '{ACTOR_PATTERN}'"); return
    actor = objs[0]
    print("Target actor:", actor)

    # Pause and capture synchronously
    client.simPause(True)
    try:
        try:
            actor_pose = client.simGetObjectPose(actor, True)
        except TypeError:
            actor_pose = client.simGetObjectPose(actor)

        cam_info = client.simGetCameraInfo(CAMERA_NAME)
        cam_pose = cam_info.pose if cam_info else None

        # Capture Scene, Segmentation, DepthPerspective together
        reqs = [
            airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.Scene,         False, True),
            airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.Segmentation,  False, True),
            airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.DepthPerspective, True,  False),
        ]
        scene_resp, seg_resp, depth_resp = client.simGetImages(reqs)

        # Decode Scene
        img = cv2.imdecode(np.frombuffer(scene_resp.image_data_uint8, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            print("Failed to decode Scene image"); return
        h, w = img.shape[:2]

        # Decode Segmentation
        seg_img = cv2.imdecode(np.frombuffer(seg_resp.image_data_uint8, np.uint8), cv2.IMREAD_COLOR)
        if seg_img is None:
            print("Failed to decode Segmentation image")
            seg_img = np.zeros((h, w, 3), np.uint8)

        # Decode DepthPerspective (float meters)
        if depth_resp.height == 0 or depth_resp.width == 0:
            print("DepthPerspective invalid size; creating zeros.")
            depth_img = np.zeros((h, w), np.float32)
        else:
            depth_flat = np.array(depth_resp.image_data_float, dtype=np.float32)
            depth_img  = depth_flat.reshape(depth_resp.height, depth_resp.width)

        # Align seg/depth to Scene resolution if needed
        if seg_img.shape[:2] != (h, w):
            seg_img = resize_to(seg_img, w, h, is_depth=False)
        if depth_img.shape[:2] != (h, w):
            depth_img = resize_to(depth_img, w, h, is_depth=True)

        # === NEARBY KEYWORD OBJECTS (≤RANGE_MAX_M) ===
        print(f"\nFiltering scene objects by keywords {KEYWORDS} within {RANGE_MAX_M:.1f} m of camera...")
        id_to_label = load_actor_map(CSV_PATH)

        nearby = []
        try:
            all_objs = client.simListSceneObjects(".*")
        except Exception as e:
            all_objs = []
            print("simListSceneObjects failed:", e)

        if cam_pose is None:
            print("Camera pose unavailable; skipping nearby-object listing.")
        else:
            for name in all_objs:
                if not name_has_keyword(name, KEYWORDS):
                    continue
                try:
                    pose = client.simGetObjectPose(name)
                except Exception:
                    continue
                if pose is None:
                    continue
                ptw = np.array([pose.position.x_val, pose.position.y_val, pose.position.z_val], dtype=float)
                rng, x_forward = cam_to_point_range(ptw, cam_pose)
                # Require in front of camera and within range
                if (x_forward > 0) and np.isfinite(rng) and (rng <= RANGE_MAX_M):
                    label = id_to_label.get(name, "")
                    nearby.append((rng, name, label))

            nearby.sort(key=lambda t: t[0])

            print(f"Found {len(nearby)} matching object(s).")
            for i, (rng, idname, label) in enumerate(nearby, 1):
                if label:
                    print(f"  [{i:02d}] {rng:6.2f} m | IDname: {idname} | label: {label}")
                else:
                    print(f"  [{i:02d}] {rng:6.2f} m | IDname: {idname}")


    finally:
        client.simPause(False)

    # === VISIBLE OBJECTS (approx) ===
    # We enumerate scene objects via simListSceneObjects(".*") and mark as visible if:
    # - the object's world position projects into the image
    # - it is in front of the camera
    # - the depth map agrees (within VISIBLE_EPS_METERS) with the object's range
    print("\nEnumerating scene objects and testing visibility (approximate)...")
    try:
        all_objs = client.simListSceneObjects(".*")
    except Exception as e:
        all_objs = []
        print("simListSceneObjects failed:", e)

    visible_names = []
    if cam_pose is not None and len(all_objs) > 0:
        # AirSim projection matrix (4x4 row-major)
        P = np.array(cam_info.proj_mat.matrix, dtype=np.float64).reshape((4,4))
        for name in all_objs:
            try:
                pose = client.simGetObjectPose(name)
            except Exception:
                continue
            if pose is None:
                continue
            pt_world = np.array([pose.position.x_val, pose.position.y_val, pose.position.z_val], dtype=float)
            if approx_visible(pt_world, cam_pose, P, w, h, depth_img, eps=VISIBLE_EPS_METERS):
                visible_names.append(name)

        print(f"Total objects in scene: {len(all_objs)}")
        print(f"Approx. visible in current {CAMERA_NAME} view: {len(visible_names)} "
              f"(eps={VISIBLE_EPS_METERS:.2f} m)")
        for i, n in enumerate(visible_names[:MAX_VISIBLE_PRINT], 1):
            print(f"  [{i:03d}] {n}")
        if len(visible_names) > MAX_VISIBLE_PRINT:
            print(f"  ... and {len(visible_names) - MAX_VISIBLE_PRINT} more.")
    else:
        print("Camera pose not available or no scene objects found.")

    # Log poses
    print_pose(f"Actor ({actor})", actor_pose)
    print_pose(f"Camera ({CAMERA_NAME})", cam_pose)

    # Apply Z-offset to actor centroid in NED from selected profile
    adjusted_actor_pose = airsim.Pose(
        position_val=airsim.Vector3r(
            actor_pose.position.x_val,
            actor_pose.position.y_val,
            actor_pose.position.z_val + Z_OFFSET_NED
        ),
        orientation_val=actor_pose.orientation
    )
    print(f"Profile: {OBJECT_TYPE} | Applied Z offset (NED): {Z_OFFSET_NED:+.4f} m")
    print_pose(f"Actor+Zoffset ({actor})", adjusted_actor_pose)

    # Intrinsics (log only)
    K, vfov = compute_intrinsics_from_horizontal_fov(cam_info.fov, w, h)
    print(f"Resolution: {w}×{h}, HFOV: {cam_info.fov:.4f}°, VFOV: {vfov:.4f}°")
    print("K =\n", K, "\n")

    # AirSim projection matrix (4x4 row-major)
    P = np.array(cam_info.proj_mat.matrix, dtype=np.float64).reshape((4,4))
    print("AirSim projection matrix P=\n", P, "\n")

    # Compute box corners (world)
    corners_w = compute_bounding_box_corners_world(
        adjusted_actor_pose, BOX_LENGTH, BOX_WIDTH, BOX_HEIGHT)
    print(f"[{OBJECT_TYPE}] Box L×W×H = {BOX_LENGTH}×{BOX_WIDTH}×{BOX_HEIGHT} m (Z-offset applied)")
    for i, c in enumerate(corners_w):
        print(f" [{i}] x={c[0]:.4f}, y={c[1]:.4f}, z={c[2]:.4f}")
    print()

    # Project & draw amodal 2D box; show ROI crops for seg/depth
    disp = img.copy()
    amodal_bbox = None
    roi_pcd = None
    roi_world_pts = None
    roi_colors_rgb = None
    best_rgb = None  # NEW: keep dominant color RGB

    if PROJECTION_ENABLED and cam_pose is not None:
        pts2d, depth_forward, valid = project_world_points_to_image(
            corners_w, cam_pose, P, w, h)

        amodal_bbox = draw_2d_bbox_and_get_rect(
            pts2d, valid, w, h, img_to_draw=disp, color=(0,255,0), thickness=2)
    else:
        print("Skipping projection.\n")

    # ROI windows + build ROI point cloud
    if SHOW_ROI_WINDOWS and amodal_bbox is not None:
        x0, y0, x1, y1 = amodal_bbox
        x0 = max(0, min(w-1, x0)); x1 = max(0, min(w-1, x1))
        y0 = max(0, min(h-1, y0)); y1 = max(0, min(h-1, y1))
        if x1 > x0 and y1 > y0:
            seg_roi   = seg_img[y0:y1+1, x0:x1+1, :]
            depth_roi = depth_img[y0:y1+1, x0:x1+1]
            depth_vis = depth_roi_to_vis(depth_roi)

            if DEPTH_CLIP_ENABLE:
                valid_depth_mask = np.isfinite(depth_roi) & (depth_roi > 0) & (depth_roi <= DEPTH_CLIP_MAX_M)
                seg_roi_clipped = np.zeros_like(seg_roi)
                seg_roi_clipped[valid_depth_mask] = seg_roi[valid_depth_mask]

                # list unique colors in clipped seg ROI
                flat = seg_roi_clipped.reshape(-1, 3)
                nonzero = np.any(flat != 0, axis=1)
                flat_nz = flat[nonzero]
                if flat_nz.size > 0:
                    colors_bgr, counts = np.unique(flat_nz, axis=0, return_counts=True)
                    colors_rgb_arr = colors_bgr[:, ::-1]
                    rgb_list = [tuple(int(c) for c in row) for row in colors_rgb_arr]
                    print(f"[Depth-clip <= {DEPTH_CLIP_MAX_M:.2f} m] Unique instance colors in seg ROI: {len(rgb_list)}")
                    print(" RGB colors:", rgb_list)
                else:
                    print(f"[Depth-clip <= {DEPTH_CLIP_MAX_M:.2f} m] Unique instance colors in seg ROI: 0")

                # back-project clipped ROI to a world point cloud with P
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
                        print(f"ROI point cloud: {world_pts.shape[0]} points added to Open3D (P-based).")
                    else:
                        print("ROI point cloud: no valid points (after clipping/stride).")
                        
                # show windows
                if SHOW_ORIGINAL_SEG_ROI:
                    cv2.namedWindow("Seg ROI (raw)", cv2.WINDOW_NORMAL)
                    cv2.imshow("Seg ROI (raw)", seg_roi)
                cv2.namedWindow(f"Seg ROI (depth <= {DEPTH_CLIP_MAX_M:.1f} m)", cv2.WINDOW_NORMAL)
                cv2.imshow(f"Seg ROI (depth <= {DEPTH_CLIP_MAX_M:.1f} m)", seg_roi_clipped)
            else:
                cv2.namedWindow("Seg ROI", cv2.WINDOW_NORMAL)
                cv2.imshow("Seg ROI", seg_roi)

            cv2.namedWindow("Depth ROI", cv2.WINDOW_NORMAL)
            cv2.imshow("Depth ROI", depth_vis)
        else:
            print("Amodal bbox collapsed after clipping; no ROI to show.")

    # Dominant colors among points inside the 3D cuboid
    if roi_world_pts is not None and roi_colors_rgb is not None:
        top_colors, n_inside = dominant_colors_in_box(
            roi_world_pts, roi_colors_rgb, adjusted_actor_pose, BOX_LENGTH, BOX_WIDTH, BOX_HEIGHT, top_k=3
        )
        print(f"Points inside 3D box: {n_inside}")
        if n_inside == 0:
            print("No ROI points fell inside the 3D cuboid.")
        else:
            if len(top_colors) > 0:
                best_rgb, best_count, best_frac = top_colors[0]
                print(f"Dominant color inside 3D box (RGB): {best_rgb}  count={best_count}  frac={best_frac:.3f}")
                if len(top_colors) > 1:
                    print("Top colors (RGB, count, frac):", top_colors)

                # --- NEW: refit the 2D bounding box tightly to this color ---
                if amodal_bbox is not None:
                    ax0, ay0, ax1, ay1 = amodal_bbox
                    margin = REFIT_SEARCH_MARGIN_PX
                    search_rect = (
                        max(0, ax0 - margin),
                        max(0, ay0 - margin),
                        min(w-1, ax1 + margin),
                        min(h-1, ay1 + margin),
                    )
                else:
                    search_rect = (0, 0, w-1, h-1)

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
                    cv2.rectangle(disp, (tx0,ty0), (tx1,ty1), (0,165,255), 2)  # orange
                    cv2.putText(disp, "tight color box", (tx0, max(0,ty0-6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,165,255), 1)
                    print(f"Tight 2D box for dominant color {best_rgb}: "
                          f"({tx0},{ty0})-({tx1},{ty1}), pixels={pix_count}")
                else:
                    print(f"Tight 2D box: dominant color {best_rgb} not found "
                          f"(min_pixels={REFIT_MIN_PIXELS}, search_rect={search_rect}).")
            else:
                print("No colors tallied (unexpected).")

    # annotate & show main window
    disp_text = f"Actor: {actor} | Profile: {OBJECT_TYPE} | Zoff(NED): {Z_OFFSET_NED:+.2f}m"
    cv2.putText(disp, disp_text, (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0), 2)
    cv2.namedWindow("Projected 3D Bounding Box", cv2.WINDOW_NORMAL)
    cv2.imshow("Projected 3D Bounding Box", disp)

    print("Press any key to exit (after Open3D closes if opened).")
    cv2.waitKey(1)  # keep small; user can view while Open3D is up

    # Open3D viz
    if o3d:
        try:
            frames = []
            Ta = np.eye(4); Ta[:3,:3] = quaternion_to_rotation_matrix(actor_pose.orientation)
            Ta[:3,3]  = [actor_pose.position.x_val, actor_pose.position.y_val, actor_pose.position.z_val]
            fa = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5); fa.transform(Ta)
            frames.append(fa)

            if cam_pose:
                Tc = np.eye(4); Tc[:3,:3] = quaternion_to_rotation_matrix(cam_pose.orientation)
                Tc[:3,3]  = [cam_pose.position.x_val, cam_pose.position.y_val, cam_pose.position.z_val]
                fc = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5); fc.transform(Tc)
                frames.append(fc)

            edges = [[0,1],[1,3],[3,2],[2,0],[4,5],[5,7],[7,6],[6,4],[0,4],[1,5],[2,6],[3,7]]
            ls = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(corners_w),
                                      lines=o3d.utility.Vector2iVector(edges))
            ls.colors = o3d.utility.Vector3dVector([[1,0,0]]*len(edges))
            frames.append(ls)

            if roi_pcd is not None:
                frames.append(roi_pcd)

            print("Showing 3D viz in Open3D.")
            o3d.visualization.draw_geometries(frames)
        except Exception as e:
            print("Open3D error:", e)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
