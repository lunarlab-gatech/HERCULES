#!/usr/bin/env python3
"""
Multi-actor version (fixed):
 - For each pattern in ACTOR_PATTERNS, pick the FIRST matching actor (same semantics as single).
 - While sim is PAUSED, capture:
     * actor poses (snapshot per matched actor)
     * camera info, images (Scene/Seg/Depth), and P
 - After unpausing, process each actor using the FROZEN pose from capture time.
 - Draw all amodal 2D boxes on one main window; show ROI/Depth windows only for the first actor.
"""

import math
import numpy as np
import setup_path
import hercules as airsim
import cv2
import csv

try:
    import open3d as o3d
except ImportError:
    o3d = None

# ===================== CONFIG =====================
ACTOR_PATTERNS = [
    # "SkeletalMeshActor_UAID.*",
    # "StaticMeshActor_UAID_E08F4CF5208AA07502_2022041209.*",
    "StaticMeshActor_UAID_E08F4CF5208AA07502_2022041209",
    "BP_VehicleAI_pickup_C_UAID_6C6E07132D49788102_1328099840",  # example
]

CAMERA_NAME  = "front_center"
CLIENT_CLASS = airsim.MultirotorClient
PORT         = 41451

PROJECTION_ENABLED = True
DRAW_2D_BBOX       = True

OBJECT_TYPE = "car"  # or "human"
PROFILES = {
    "human": {"L": 0.5, "W": 0.75, "H": 1.9,  "Z": -0.90},
    "car":   {"L": 4.2, "W": 1.90, "H": 1.60, "Z": -0.55},
}
if OBJECT_TYPE not in PROFILES:
    raise ValueError(f"Unknown OBJECT_TYPE '{OBJECT_TYPE}'.")

BOX_LENGTH   = PROFILES[OBJECT_TYPE]["L"]
BOX_WIDTH    = PROFILES[OBJECT_TYPE]["W"]
BOX_HEIGHT   = PROFILES[OBJECT_TYPE]["H"]
Z_OFFSET_NED = PROFILES[OBJECT_TYPE]["Z"]

SHOW_ROI_WINDOWS   = True
DEPTH_CLIP_ENABLE  = True
DEPTH_CLIP_MAX_M   = 40.0
SHOW_ORIGINAL_SEG_ROI = True

ADD_ROI_POINTS_TO_OPEN3D = True
ROI_POINT_STRIDE         = 1

REFIT_USE_DEPTH_CLIP_FOR_TIGHT_BOX = True
REFIT_MIN_PIXELS                   = 50
REFIT_SEARCH_MARGIN_PX             = 20

VISIBLE_EPS_METERS = 1.0
MAX_VISIBLE_PRINT  = 200

CSV_PATH    = "/home/sgarimella34/multi-robot-coordination/HERCULES/csv_data/ue_label_vs_name.csv"
KEYWORDS    = ("human", "car", "truck", "sedan", "suv", "vehicle")
RANGE_MAX_M = 40.0

# ===================== UTILS (unchanged from single) =====================
def load_actor_map(csv_path):
    mapping = {}
    try:
        with open(csv_path, "r", newline="") as f:
            sniffer = csv.Sniffer()
            sample = f.read(1024); f.seek(0)
            has_header = sniffer.has_header(sample)
            reader = csv.reader(f)
            if has_header:
                next(reader, None)
            for row in reader:
                if not row or len(row) < 2: continue
                actor_label = row[0].strip()
                idname = row[1].strip()
                if idname: mapping[idname] = actor_label
    except FileNotFoundError:
        print(f"CSV not found: {csv_path} (continuing without labels)")
    except Exception as e:
        print(f"Error reading CSV '{csv_path}': {e} (continuing without labels)")
    return mapping

def name_has_keyword(idname, keywords):
    s = idname.lower()
    return any(k.lower() in s for k in keywords)

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
        print(f"{label}: <no pose>"); return
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
        sampled = np.zeros_like(mask); sampled[::stride, ::stride] = True
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

def cam_to_point_range(pt_world, cam_pose):
    R_cam = quaternion_to_rotation_matrix(cam_pose.orientation)
    cam_p = np.array([cam_pose.position.x_val, cam_pose.position.y_val, cam_pose.position.z_val], dtype=float)
    p_cam = R_cam.T @ (pt_world - cam_p)
    return float(np.linalg.norm(p_cam)), p_cam[0]

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
    if not (in_front and in_bounds): return False
    r_obj = float(np.linalg.norm(p_cam))
    if not np.isfinite(r_obj) or r_obj <= 0: return False
    ui, vi = int(round(u)), int(round(v))
    ui = max(0, min(img_w - 1, ui)); vi = max(0, min(img_h - 1, vi))
    r_depth = float(depth_img[vi, ui]) if depth_img is not None else float("inf")
    if not np.isfinite(r_depth) or r_depth <= 0: return False
    return abs(r_depth - r_obj) <= eps

# ===================== PER-ACTOR PIPELINE =====================
def process_one_actor(actor_name, actor_pose_snap, cam_pose, img, seg_img, depth_img,
                      P, w, h, show_roi_windows_for_this_actor,
                      object_type, box_L, box_W, box_H, z_offset_ned,
                      add_roi_to_o3d=True, roi_stride=1,
                      refit_use_depth=True, refit_min_pixels=50, refit_margin_px=20,
                      depth_clip_enable=True, depth_clip_max_m=40.0):
    """
    Use the SNAPSHOT pose (actor_pose_snap) taken while paused (same time as images).
    """
    out = {"disp_annotations": [], "o3d_geoms": []}

    print_pose(f"Actor ({actor_name}) [snapshot]", actor_pose_snap)

    adjusted_actor_pose = airsim.Pose(
        position_val=airsim.Vector3r(
            actor_pose_snap.position.x_val,
            actor_pose_snap.position.y_val,
            actor_pose_snap.position.z_val + z_offset_ned
        ),
        orientation_val=actor_pose_snap.orientation
    )
    print(f"Profile: {object_type} | Applied Z offset (NED): {z_offset_ned:+.4f} m")
    print_pose(f"Actor+Zoffset ({actor_name}) [snapshot]", adjusted_actor_pose)

    corners_w = compute_bounding_box_corners_world(adjusted_actor_pose, box_L, box_W, box_H)
    print(f"[{object_type}] Box L×W×H = {box_L}×{box_W}×{box_H} m (Z-offset applied)")
    for i, c in enumerate(corners_w):
        print(f" [{i}] x={c[0]:.4f}, y={c[1]:.4f}, z={c[2]:.4f}")
    print()

    amodal_bbox = None
    if PROJECTION_ENABLED and cam_pose is not None:
        pts2d, depth_forward, valid = project_world_points_to_image(corners_w, cam_pose, P, w, h)
        amodal_bbox = draw_2d_bbox_and_get_rect(pts2d, valid, w, h, img_to_draw=None, color=(0,255,0), thickness=2)
        if amodal_bbox is not None and DRAW_2D_BBOX:
            out["disp_annotations"].append(("rect", (*amodal_bbox, (0,255,0), 2)))

    roi_world_pts = None
    roi_colors_rgb = None
    roi_pcd = None

    if show_roi_windows_for_this_actor and amodal_bbox is not None:
        x0, y0, x1, y1 = amodal_bbox
        x0 = max(0, min(w-1, x0)); x1 = max(0, min(w-1, x1))
        y0 = max(0, min(h-1, y0)); y1 = max(0, min(h-1, y1))
        if x1 > x0 and y1 > y0:
            seg_roi   = seg_img[y0:y1+1, x0:x1+1, :]
            depth_roi = depth_img[y0:y1+1, x0:x1+1]
            depth_vis = depth_roi_to_vis(depth_roi)

            if depth_clip_enable:
                valid_depth_mask = np.isfinite(depth_roi) & (depth_roi > 0) & (depth_roi <= depth_clip_max_m)
                seg_roi_clipped = np.zeros_like(seg_roi)
                seg_roi_clipped[valid_depth_mask] = seg_roi[valid_depth_mask]

                if add_roi_to_o3d and o3d is not None:
                    world_pts, colors = roi_points_to_world_pointcloud_P(
                        seg_roi_clipped, depth_roi, x0, y0, w, h, P, cam_pose, stride=roi_stride
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
                        print("ROI point cloud: no valid points after clipping/stride.")

                if SHOW_ORIGINAL_SEG_ROI:
                    cv2.namedWindow("Seg ROI (raw)", cv2.WINDOW_NORMAL)
                    cv2.imshow("Seg ROI (raw)", seg_roi)
                cv2.namedWindow(f"Seg ROI (depth <= {depth_clip_max_m:.1f} m)", cv2.WINDOW_NORMAL)
                cv2.imshow(f"Seg ROI (depth <= {depth_clip_max_m:.1f} m)", seg_roi_clipped)
            else:
                cv2.namedWindow("Seg ROI", cv2.WINDOW_NORMAL)
                cv2.imshow("Seg ROI", seg_roi)

            cv2.namedWindow("Depth ROI", cv2.WINDOW_NORMAL)
            cv2.imshow("Depth ROI", depth_vis)

    # Dominant color & tight refit (only if we built ROI points)
    if roi_world_pts is not None and roi_colors_rgb is not None:
        top_colors, n_inside = dominant_colors_in_box(
            roi_world_pts, roi_colors_rgb, adjusted_actor_pose, box_L, box_W, box_H, top_k=3
        )
        print(f"Points inside 3D box: {n_inside}")
        if n_inside > 0 and len(top_colors) > 0:
            best_rgb, best_count, best_frac = top_colors[0]
            print(f"Dominant color inside 3D box (RGB): {best_rgb}  count={best_count}  frac={best_frac:.3f}")
            if amodal_bbox is not None:
                ax0, ay0, ax1, ay1 = amodal_bbox
                margin = refit_margin_px
                search_rect = (
                    max(0, ax0 - margin), max(0, ay0 - margin),
                    min(w-1, ax1 + margin), min(h-1, ay1 + margin),
                )
            else:
                search_rect = (0, 0, w-1, h-1)
            tight_box, pix_count = tight_box_for_color(
                seg_img, depth_img, best_rgb, search_rect=search_rect,
                use_depth=refit_use_depth and depth_clip_enable,
                depth_max=depth_clip_max_m, min_pixels=refit_min_pixels
            )
            if tight_box is not None:
                out["disp_annotations"].append(("rect", (*tight_box, (0,165,255), 2)))
                out["disp_annotations"].append(("text", (tight_box[0], max(0, tight_box[1]-6),
                                                         "tight color box", 0.5, (0,165,255), 1)))
                print(f"Tight 2D box for dominant color {best_rgb}: {tight_box}, pixels={pix_count}")

    # Open3D geoms
    if o3d:
        try:
            Ta = np.eye(4)
            Ta[:3,:3] = quaternion_to_rotation_matrix(actor_pose_snap.orientation)
            Ta[:3,3]  = [actor_pose_snap.position.x_val, actor_pose_snap.position.y_val, actor_pose_snap.position.z_val]
            fa = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5); fa.transform(Ta)
            out["o3d_geoms"].append(fa)

            edges = [[0,1],[1,3],[3,2],[2,0],[4,5],[5,7],[7,6],[6,4],[0,4],[1,5],[2,6],[3,7]]
            ls = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(corners_w),
                                      lines=o3d.utility.Vector2iVector(edges))
            ls.colors = o3d.utility.Vector3dVector([[1,0,0]]*len(edges))
            out["o3d_geoms"].append(ls)

            if roi_pcd is not None:
                out["o3d_geoms"].append(roi_pcd)
        except Exception as e:
            print("Open3D error (actor geoms):", e)

    return out

# ===================== MAIN =====================
def main():
    np.set_printoptions(precision=4, suppress=True)
    client = CLIENT_CLASS(port=PORT)
    client.confirmConnection()
    print("Connected!\n")

    dparams = client.simGetDistortionParams(CAMERA_NAME)
    print("Distortion params:", dparams)
    if any(abs(d)>1e-9 for d in dparams):
        print(" Zeroing distortion.")
        client.simSetDistortionParams(CAMERA_NAME, {"K1":0.0, "K2":0.0, "K3":0.0, "P1":0.0, "P2":0.0})
    else:
        print(" No distortion active.")
    print()

    # ---------- PAUSE: snapshot camera, resolve actors, snapshot their poses, capture images ----------
    client.simPause(True)
    try:
        cam_info = client.simGetCameraInfo(CAMERA_NAME)
        cam_pose = cam_info.pose if cam_info else None

        # Resolve first match per pattern WHILE PAUSED (names + snapshot poses)
        resolved = []  # list of (pattern, actor_name, actor_pose_snapshot)
        for patt in ACTOR_PATTERNS:
            names = client.simListSceneObjects(patt)
            if not names:
                print(f"No actor matches '{patt}'"); continue
            actor = names[0]
            try:
                actor_pose = client.simGetObjectPose(actor, True)
            except TypeError:
                actor_pose = client.simGetObjectPose(actor)
            resolved.append((patt, actor, actor_pose))
        if not resolved:
            print("No actors resolved; quitting.")
            return

        # Capture images at the SAME paused time
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
            print("Failed to decode Segmentation image"); seg_img = np.zeros((h, w, 3), np.uint8)

        if depth_resp.height == 0 or depth_resp.width == 0:
            print("DepthPerspective invalid size; creating zeros.")
            depth_img = np.zeros((h, w), np.float32)
        else:
            depth_flat = np.array(depth_resp.image_data_float, dtype=np.float32)
            depth_img  = depth_flat.reshape(depth_resp.height, depth_resp.width)

        if seg_img.shape[:2] != (h, w): seg_img = resize_to(seg_img, w, h, is_depth=False)
        if depth_img.shape[:2] != (h, w): depth_img = resize_to(depth_img, w, h, is_depth=True)

        # Intrinsics/projection (log once)
        K, vfov = compute_intrinsics_from_horizontal_fov(cam_info.fov, w, h)
        print(f"Resolution: {w}×{h}, HFOV: {cam_info.fov:.4f}°, VFOV: {vfov:.4f}°")
        print("K =\n", K, "\n")
        P = np.array(cam_info.proj_mat.matrix, dtype=np.float64).reshape((4,4))
        print("AirSim projection matrix P=\n", P, "\n")

        # Nearby keyword objects listing at the same snapshot
        print(f"\nFiltering scene objects by keywords {KEYWORDS} within {RANGE_MAX_M:.1f} m of camera...")
        id_to_label = load_actor_map(CSV_PATH)
        try:
            all_objs = client.simListSceneObjects(".*")
        except Exception as e:
            all_objs = []; print("simListSceneObjects failed:", e)

        if cam_pose is None:
            print("Camera pose unavailable; skipping nearby-object listing.")
        else:
            nearby = []
            for name in all_objs:
                if not name_has_keyword(name, KEYWORDS): continue
                try:
                    pose = client.simGetObjectPose(name)
                except Exception:
                    continue
                if pose is None: continue
                ptw = np.array([pose.position.x_val, pose.position.y_val, pose.position.z_val], dtype=float)
                rng, x_forward = cam_to_point_range(ptw, cam_pose)
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
        client.simPause(False)  # unpause AFTER we have images + poses

    # ---------- Visibility check (approx) uses the same frozen P and the captured depth ----------
    print("\nEnumerating scene objects and testing visibility (approximate)...")
    try:
        all_objs = client.simListSceneObjects(".*")
    except Exception as e:
        all_objs = []; print("simListSceneObjects failed:", e)

    if cam_pose is not None and len(all_objs) > 0:
        visible_names = []
        for name in all_objs:
            try:
                pose = client.simGetObjectPose(name)
            except Exception:
                continue
            if pose is None: continue
            pt_world = np.array([pose.position.x_val, pose.position.y_val, pose.position.z_val], dtype=float)
            if approx_visible(pt_world, cam_pose, P, w, h, depth_img, eps=VISIBLE_EPS_METERS):
                visible_names.append(name)
        print(f"Total objects in scene: {len(all_objs)}")
        print(f"Approx. visible in current {CAMERA_NAME} view: {len(visible_names)} (eps={VISIBLE_EPS_METERS:.2f} m)")
        for i, n in enumerate(visible_names[:MAX_VISIBLE_PRINT], 1):
            print(f"  [{i:03d}] {n}")
        if len(visible_names) > MAX_VISIBLE_PRINT:
            print(f"  ... and {len(visible_names) - MAX_VISIBLE_PRINT} more.")
    else:
        print("Camera pose not available or no scene objects found.")

    # ---------- Shared display + Open3D aggregation ----------
    disp = img.copy()
    o3d_geoms = []
    if o3d and cam_pose is not None:
        try:
            Tc = np.eye(4); Tc[:3,:3] = quaternion_to_rotation_matrix(cam_pose.orientation)
            Tc[:3,3]  = [cam_pose.position.x_val, cam_pose.position.y_val, cam_pose.position.z_val]
            fc = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5); fc.transform(Tc)
            o3d_geoms.append(fc)
        except Exception as e:
            print("Open3D error (camera frame):", e)

    first_actor_roi_done = False

    # ---------- Process each resolved actor using its SNAPSHOT pose ----------
    for patt, actor_name, actor_pose_snap in resolved:
        print("\n==============================")
        print(f"Processing target actor: {actor_name} (pattern: {patt})")
        print("==============================")

        out = process_one_actor(
            actor_name=actor_name,
            actor_pose_snap=actor_pose_snap,
            cam_pose=cam_pose,
            img=img, seg_img=seg_img, depth_img=depth_img,
            P=P, w=w, h=h,
            show_roi_windows_for_this_actor=(SHOW_ROI_WINDOWS and not first_actor_roi_done),
            object_type=OBJECT_TYPE,
            box_L=BOX_LENGTH, box_W=BOX_WIDTH, box_H=BOX_HEIGHT,
            z_offset_ned=Z_OFFSET_NED,
            add_roi_to_o3d=ADD_ROI_POINTS_TO_OPEN3D,
            roi_stride=ROI_POINT_STRIDE,
            refit_use_depth=REFIT_USE_DEPTH_CLIP_FOR_TIGHT_BOX,
            refit_min_pixels=REFIT_MIN_PIXELS,
            refit_margin_px=REFIT_SEARCH_MARGIN_PX,
            depth_clip_enable=DEPTH_CLIP_ENABLE,
            depth_clip_max_m=DEPTH_CLIP_MAX_M
        )

        # draw queued annotations for this actor onto shared display
        for kind, params in out["disp_annotations"]:
            if kind == "rect":
                x0, y0, x1, y1, color, thick = params
                cv2.rectangle(disp, (x0,y0), (x1,y1), color, thick)
            elif kind == "text":
                x, y, text, scale, color, thick = params
                cv2.putText(disp, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick)

        if o3d and out["o3d_geoms"]:
            o3d_geoms.extend(out["o3d_geoms"])

        if SHOW_ROI_WINDOWS and not first_actor_roi_done:
            first_actor_roi_done = True

    # ---------- Final display ----------
    disp_text = f"Actors: {len(resolved)} | Profile: {OBJECT_TYPE} | Zoff(NED): {Z_OFFSET_NED:+.2f}m"
    cv2.putText(disp, disp_text, (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0), 2)
    cv2.namedWindow("Projected 3D Bounding Box", cv2.WINDOW_NORMAL)
    cv2.imshow("Projected 3D Bounding Box", disp)

    print("Press any key to exit (after Open3D closes if opened).")
    cv2.waitKey(1)

    if o3d and o3d_geoms:
        try:
            print("Showing 3D viz in Open3D.")
            o3d.visualization.draw_geometries(o3d_geoms)
        except Exception as e:
            print("Open3D error:", e)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

# small helper used above
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

if __name__ == "__main__":
    main()
