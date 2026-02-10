#!/usr/bin/env python3
"""
collect_sim_to_dair_kitti.py

Robust data collection + labeling pipeline for Hercules (HERCULES) simulator
to produce DAIR-V2X / KITTI-compatible data.

Labels are generated in the same tick as the sensor data capture to preserve sync.
This version mirrors the exact detection logic from 2D_and_3D_object_detection.py:
- keyword filtering (CSV + idname)
- profile selection and overrides
- amodal 3D->2D projection (cuboid)
- ROI back-projection with depth clip
- dominant-color inside-3D-box selection
- tight 2D box refit with occlusion/small-box gating

Both vehicle and infrastructure cameras are sampled on the same paused-sim tick.
"""

import os, re, csv, json, math, time, traceback
from collections import deque
import numpy as np
import cv2

import setup_path  # ensure AirSim / HERCULES is on PYTHONPATH
import hercules as airsim  # your custom fork

# ------------------ USER CONFIG ------------------

SETTINGS_JSON_PATH = "/home/sgarimella34/Documents/AirSim/settings.json"
OUT_ROOT = "/home/sgarimella34/multi-robot-coordination/collaborative-perception-BEVP/datasets/dair_v2x_synth"

SIDE_INF = ["Drone1"]   # infrastructure proxy
SIDE_VEH = ["Husky1"]   # vehicle proxy

CAM_NAME   = "front_center"
LIDAR_NAME = "LidarSensor1"

IMG_EXT = ".png"

# DAIR-V2X-C style: both at 10 Hz
CAM_RATE   = 10
LIDAR_RATE = 10
DURATION_S = 300.0

BASE_HZ    = CAM_RATE
DT         = 1.0 / BASE_HZ
CAM_EVERY  = BASE_HZ // CAM_RATE
LIDAR_EVERY= BASE_HZ // LIDAR_RATE

VEHICLE_DET_RADIUS_CM = int(120 * 100)
INF_DET_RADIUS_CM     = int(300 * 100)

DRONE_PORT = 41451
HUSKY_PORT = 41452

# --- mapping csv + filters (copied from detection script) ---
CSV_PATH = "/home/sgarimella34/multi-robot-coordination/HERCULES/csv_data/ue_label_vs_name.csv"
KEYWORDS = (
    "human", "person", "pedestrian",
    "car", "truck", "sedan", "suv", "van", "bus", "vehicle",
    "sportscar", "sports car", "policecar", "pickup", "taxi"
)
MAX_OBJECTS = 200

# --- bbox gating & depth-clip (same defaults as detection) ---
RANGE_MAX_M = 40.0
DEPTH_CLIP_ENABLE = True
DEPTH_CLIP_MAX_M  = RANGE_MAX_M
REFIT_USE_DEPTH_CLIP_FOR_TIGHT_BOX = True
REFIT_MIN_PIXELS   = 50
REFIT_SEARCH_MARGIN_PX = 20
MIN_BBOX_WIDTH  = 20
MIN_BBOX_HEIGHT = 20
MIN_BBOX_AREA   = 400
ROI_POINT_STRIDE = 1

# Additional object profile definitions (L,W,H,Z in meters; Z is NED +Z down)
PROFILES = {
    "human": {"L": 0.5, "W": 0.75, "H": 1.9,  "Z": -0.90},
    "car":   {"L": 4.2, "W": 1.90, "H": 1.60, "Z": -0.55},
}
# Optional per-ID overrides
PROFILE_OVERRIDES_BY_ID_SUBSTR = {
    "pickup": {"L": 5.35, "W": 2.05, "H": 2.2, "Z": -1.0, "FWD_OFF_M": -0.45},
}

# Wildcard patterns for AirSim detection filters (keeps detection list small; final filtering still uses KEYWORDS)
MESH_FILTERS = {
    "veh": ["Sportscar", "Sedan2", "SK_Survival_Character"],
    "inf": ["Sportscar", "Sedan2", "SK_Survival_Character"],
}

# Flip matrix to convert Z-down to Z-up virtual-lidar frame for yaw extraction
F_VL = np.diag([1.0, 1.0, -1.0])

# ------------------ OUTPUT PATHS ------------------

def dair_paths(root: str):
    return {
        "inf": {
            "img":         f"{root}/cooperative/infrastructure-side/image",
            "depth":       f"{root}/cooperative/infrastructure-side/depth",
            "seg":         f"{root}/cooperative/infrastructure-side/seg",
            "lidar":       f"{root}/cooperative/infrastructure-side/lidar",
            "calib":       f"{root}/cooperative/infrastructure-side/calib",
            "ts":          f"{root}/cooperative/infrastructure-side/timestamp",
            "kitti_label": f"{root}/cooperative/infrastructure-side/kitti_label"
        },
        "veh": {
            "img":         f"{root}/cooperative/vehicle-side/image",
            "depth":       f"{root}/cooperative/vehicle-side/depth",
            "seg":         f"{root}/cooperative/vehicle-side/seg",
            "lidar":       f"{root}/cooperative/vehicle-side/lidar",
            "calib":       f"{root}/cooperative/vehicle-side/calib",
            "ts":          f"{root}/cooperative/vehicle-side/timestamp",
            "kitti_label": f"{root}/cooperative/vehicle-side/kitti_label"
        },
        "label": {
            "veh":         f"{root}/cooperative/label/vehicle",
            "inf":         f"{root}/cooperative/label/infrastructure",
            "cooperative": f"{root}/cooperative/label/cooperative"
        }
    }

PATHS = dair_paths(OUT_ROOT)
for grp in PATHS.values():
    for p in grp.values():
        os.makedirs(p, exist_ok=True)

# ------------------ AIRSIM CLIENTS ------------------

drone_client = airsim.MultirotorClient(port=DRONE_PORT)
car_client   = airsim.CarClient(port=HUSKY_PORT)

print("[STARTUP] connecting to AirSim clients...")
drone_client.confirmConnection()
car_client.confirmConnection()
print("[STARTUP] connected to AirSim clients")

for n in SIDE_INF:
    drone_client.enableApiControl(True, vehicle_name=n)
for n in SIDE_VEH:
    car_client.enableApiControl(True, vehicle_name=n)

# Pause world; we will step deterministically with simContinueForTime
drone_client.simPause(True)

# ------------------ UTILS / GEOMETRY (shared with your detection script) ------------------

def load_settings(path):
    with open(path, "r") as f:
        return json.load(f)

def euler_to_rot_matrix(yaw_deg, pitch_deg, roll_deg):
    yaw   = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    roll  = math.radians(roll_deg)
    Rz = np.array([[ math.cos(yaw),-math.sin(yaw),0],
                   [ math.sin(yaw), math.cos(yaw),0],
                   [ 0, 0, 1]], float)
    Ry = np.array([[ math.cos(pitch),0, math.sin(pitch)],
                   [ 0, 1, 0],
                   [-math.sin(pitch),0, math.cos(pitch)]], float)
    Rx = np.array([[1,0,0],
                   [0, math.cos(roll),-math.sin(roll)],
                   [0, math.sin(roll), math.cos(roll)]], float)
    return Rz @ Ry @ Rx

def quat_to_rot_matrix(w, x, y, z):
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n == 0: return np.eye(3, dtype=float)
    w,x,y,z = w/n, x/n, y/n, z/n
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y - z*w),  2*(x*z + y*w)],
        [2*(x*y + z*w),   1-2*(x*x+z*z),  2*(y*z - x*w)],
        [2*(x*z - y*w),   2*(y*z + x*w),  1-2*(x*x+y*y)]
    ], dtype=float)

def build_homogeneous(R, t):
    T = np.eye(4, dtype=float)
    T[:3,:3] = R
    T[:3,3]  = t
    return T

def invert_homogeneous(T):
    R = T[:3,:3]; t=T[:3,3]
    Ti = np.eye(4, dtype=float)
    Ti[:3,:3] = R.T
    Ti[:3,3]  = -R.T @ t
    return Ti

def extract_yaw_from_virtual_lidar_rotation(R):
    return math.atan2(R[1,0], R[0,0])

def compute_intrinsics_from_horizontal_fov(hfov_deg, width, height):
    hfov = math.radians(hfov_deg)
    fx = (width/2.0) / math.tan(hfov/2.0)
    fy = fx
    cx, cy = width/2.0, height/2.0
    K = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]], dtype=float)
    return K

def projection_from_K(K):
    # Matches your detection math (P used for NDC projection and back-projection)
    fx, fy = K[0,0], K[1,1]
    return np.array([
        [0.0, -1.0/fx, 0.0, 0.0],
        [0.0, 0.0, -1.0/fy, 0.0],
        [0.0, 0.0,  1.0,    0.0],
        [1.0, 0.0,  0.0,    0.0],
    ], dtype=float)

def get_vehicle_world_transform(client, vehicle_name, is_drone: bool):
    st = client.getMultirotorState(vehicle_name=vehicle_name) if is_drone else client.getCarState(vehicle_name=vehicle_name)
    kin = st.kinematics_estimated
    pos, ori = kin.position, kin.orientation
    R = quat_to_rot_matrix(ori.w_val, ori.x_val, ori.y_val, ori.z_val)
    return build_homogeneous(R, np.array([pos.x_val, pos.y_val, pos.z_val], dtype=float))

def get_object_world_pose(client, name):
    try:
        pose = client.simGetObjectPose(name, True)
    except TypeError:
        pose = client.simGetObjectPose(name)
    if pose is None: return None
    return pose

def world_to_cam_and_pixel(pt_world, cam_pose, P, img_w, img_h):
    R_cam = quat_to_rot_matrix(cam_pose.orientation.w_val, cam_pose.orientation.x_val,
                               cam_pose.orientation.y_val, cam_pose.orientation.z_val)
    cam_p = np.array([cam_pose.position.x_val, cam_pose.position.y_val, cam_pose.position.z_val], dtype=float)
    p_cam = R_cam.T @ (pt_world - cam_p)
    in_front = p_cam[0] > 1e-6
    pts_h = np.array([[p_cam[0], p_cam[1], p_cam[2], 1.0]], dtype=float)
    clip  = (P @ pts_h.T).T
    ndc   = (clip[:, :3] / clip[:, 3:4])[0] if clip[0,3] != 0 else np.array([0,0,0], dtype=float)
    u = (1.0 - (ndc[0] * 0.5 + 0.5)) * img_w
    v = (ndc[1] * 0.5 + 0.5) * img_h
    in_bounds = (0 <= u < img_w) and (0 <= v < img_h)
    return p_cam, float(u), float(v), bool(in_front), bool(in_bounds)

def compute_bounding_box_corners_world(pose, L, W, H):
    hl, hw, hh = L/2.0, W/2.0, H/2.0
    corners_local = np.array([
        [ hl,  hw,  hh], [ hl,  hw, -hh],
        [ hl, -hw,  hh], [ hl, -hw, -hh],
        [-hl,  hw,  hh], [-hl,  hw, -hh],
        [-hl, -hw,  hh], [-hl, -hw, -hh],
    ], dtype=float)
    R = quat_to_rot_matrix(pose.orientation.w_val, pose.orientation.x_val,
                           pose.orientation.y_val, pose.orientation.z_val)
    t = np.array([pose.position.x_val, pose.position.y_val, pose.position.z_val], dtype=float)
    return (R @ corners_local.T).T + t

def project_world_points_to_image(world_pts, cam_pose, P, width, height):
    R_cam = quat_to_rot_matrix(cam_pose.orientation.w_val, cam_pose.orientation.x_val,
                               cam_pose.orientation.y_val, cam_pose.orientation.z_val)
    cam_p = np.array([cam_pose.position.x_val, cam_pose.position.y_val, cam_pose.position.z_val], dtype=float)
    cam_pts = (R_cam.T @ (world_pts - cam_p).T).T
    pts_h = np.hstack([cam_pts, np.ones((cam_pts.shape[0],1), dtype=float)])
    clip  = (P @ pts_h.T).T
    w_comp= clip[:, 3:4]
    ndc   = clip[:, :3] / w_comp
    u = (1.0 - (ndc[:,0]*0.5 + 0.5)) * width
    v = (ndc[:,1]*0.5 + 0.5) * height
    pts2d = np.stack([u, v], axis=1)
    depth_forward = cam_pts[:, 0]
    valid = depth_forward > 1e-6
    return pts2d, depth_forward, valid

def _clip_segment_to_rect(p0, p1, w, h):
    (x0,y0),(x1,y1) = p0, p1
    x0 = float(x0); y0=float(y0); x1=float(x1); y1=float(y1)
    p = [-(x1-x0), (x1-x0), -(y1-y0), (y1-y0)]
    q = [x0-0, (w-1)-x0, y0-0, (h-1)-y0]
    u1, u2 = 0.0, 1.0
    for pi, qi in zip(p, q):
        if pi == 0:
            if qi < 0: return None
        else:
            t = qi / pi
            if pi < 0:
                if t > u2: return None
                if t > u1: u1 = t
            else:
                if t < u1: return None
                if t < u2: u2 = t
    nx0 = x0 + u1*(x1-x0); ny0 = y0 + u1*(y1-y0)
    nx1 = x0 + u2*(x1-x0); ny1 = y0 + u2*(y1-y0)
    return (nx0,ny0),(nx1,ny1)

def amodal_bbox_for_actor_with_dims(pose, cam_pose, P, img_w, img_h, L, W, H, z_off, fwd_off=0.0):
    adj_pose = pose_with_offsets(pose, z_off_m=z_off, fwd_off_m=fwd_off)
    corners_w = compute_bounding_box_corners_world(adj_pose, L, W, H)
    pts2d, depth_forward, valid = project_world_points_to_image(corners_w, cam_pose, P, img_w, img_h)
    u, v = pts2d[:,0], pts2d[:,1]
    in_bounds = (u>=0) & (u<img_w) & (v>=0) & (v<img_h)
    use = valid & in_bounds
    if np.any(use):
        x0 = int(max(0, math.floor(u[use].min())))
        y0 = int(max(0, math.floor(v[use].min())))
        x1 = int(min(img_w-1, math.ceil(u[use].max())))
        y1 = int(min(img_h-1, math.ceil(v[use].max())))
        if x1>x0 and y1>y0:
            return (x0,y0,x1,y1), True, corners_w, adj_pose, (L,W,H), depth_forward, valid
    # edge-aware fallback
    edges = [(0,1),(1,3),(3,2),(2,0),(4,5),(5,7),(7,6),(6,4),(0,4),(1,5),(2,6),(3,7)]
    clipped = []
    for a,b in edges:
        if not (valid[a] and valid[b]): continue
        seg = _clip_segment_to_rect((u[a],v[a]), (u[b],v[b]), img_w, img_h)
        if seg is not None:
            p0,p1 = seg; clipped.append(p0); clipped.append(p1)
    if len(clipped)>=2:
        cps = np.array(clipped, dtype=float)
        x0 = int(max(0, math.floor(cps[:,0].min())))
        y0 = int(max(0, math.floor(cps[:,1].min())))
        x1 = int(min(img_w-1, math.ceil(cps[:,0].max())))
        y1 = int(min(img_h-1, math.ceil(cps[:,1].max())))
        if x1>x0 and y1>y0:
            return (x0,y0,x1,y1), True, corners_w, adj_pose, (L,W,H), depth_forward, valid
    return None, True, corners_w, adj_pose, (L,W,H), depth_forward, valid

def points_inside_oriented_box(world_pts, box_pose, L, W, H, eps=1e-6):
    if world_pts is None or world_pts.size==0: return np.zeros((0,), dtype=bool)
    R = quat_to_rot_matrix(box_pose.orientation.w_val, box_pose.orientation.x_val,
                           box_pose.orientation.y_val, box_pose.orientation.z_val)
    t = np.array([box_pose.position.x_val, box_pose.position.y_val, box_pose.position.z_val], dtype=float)
    p_local = (R.T @ (world_pts - t).T).T
    hl, hw, hh = L/2.0+eps, W/2.0+eps, H/2.0+eps
    return (np.abs(p_local[:,0])<=hl) & (np.abs(p_local[:,1])<=hw) & (np.abs(p_local[:,2])<=hh)

def roi_points_to_world_pointcloud_P(seg_roi_clipped, depth_roi, x0, y0, img_w, img_h, P, cam_pose, stride=1):
    H, W = depth_roi.shape
    seg_nonzero  = np.any(seg_roi_clipped!=0, axis=2)
    finite_depth = np.isfinite(depth_roi) & (depth_roi>0)
    mask = seg_nonzero & finite_depth
    if not np.any(mask): return None, None
    if stride>1:
        sampled = np.zeros_like(mask); sampled[::stride, ::stride] = True
        mask = mask & sampled
        if not np.any(mask): return None, None
    ys, xs = np.where(mask)
    u = (x0 + xs).astype(np.float64)
    v = (y0 + ys).astype(np.float64)
    ndc_x = 1.0 - 2.0*(u/float(img_w))
    ndc_y = 2.0*(v/float(img_h)) - 1.0
    p01 = float(P[0,1]); p12 = float(P[1,2])
    if abs(p01)<1e-9 or abs(p12)<1e-9: return None, None
    rat_y = -ndc_x / p01
    rat_z = -ndc_y / p12
    dirs = np.stack([np.ones_like(rat_y), rat_y, rat_z], axis=1)
    norms= np.linalg.norm(dirs, axis=1, keepdims=True)
    dirs_unit = dirs / np.maximum(norms, 1e-12)
    r = depth_roi[ys, xs].astype(np.float64)[:,None]
    p_cam = dirs_unit * r
    R_cam = quat_to_rot_matrix(cam_pose.orientation.w_val, cam_pose.orientation.x_val,
                               cam_pose.orientation.y_val, cam_pose.orientation.z_val)
    cam_p = np.array([cam_pose.position.x_val, cam_pose.position.y_val, cam_pose.position.z_val], dtype=float)
    world_pts = (R_cam @ p_cam.T).T + cam_p
    colors_bgr = seg_roi_clipped[ys, xs, :]
    colors_rgb = colors_bgr[:, ::-1].astype(np.float32)/255.0
    return world_pts.astype(np.float64), colors_rgb

def dominant_colors_in_box(world_pts, colors_rgb, box_pose, L, W, H, top_k=3):
    if world_pts is None or colors_rgb is None or world_pts.shape[0]==0:
        return [], 0
    inside = points_inside_oriented_box(world_pts, box_pose, L, W, H)
    n_inside = int(np.count_nonzero(inside))
    if n_inside==0: return [], 0
    cols = (np.rint(colors_rgb[inside]*255.0)).astype(np.uint8)
    uniq, counts = np.unique(cols, axis=0, return_counts=True)
    order = np.argsort(-counts)
    uniq, counts = uniq[order], counts[order]
    results = []
    for i in range(min(top_k, uniq.shape[0])):
        r,g,b = [int(v) for v in uniq[i]]
        frac  = float(counts[i]) / float(n_inside)
        results.append(((r,g,b), int(counts[i]), frac))
    return results, n_inside

def tight_box_for_color(seg_img, depth_img, target_rgb, search_rect, use_depth=True, depth_max=35.0, min_pixels=50):
    h, w = seg_img.shape[:2]
    x0, y0, x1, y1 = search_rect
    x0 = max(0, min(w-1, x0)); x1 = max(0, min(w-1, x1))
    y0 = max(0, min(h-1, y0)); y1 = max(0, min(h-1, y1))
    if x1<=x0 or y1<=y0: return None, 0
    roi_seg = seg_img[y0:y1+1, x0:x1+1, :]
    color_bgr = np.array(target_rgb[::-1], np.uint8)
    mask_color = np.all(roi_seg == color_bgr, axis=2)
    if use_depth and depth_img is not None:
        roi_depth = depth_img[y0:y1+1, x0:x1+1]
        depth_ok  = np.isfinite(roi_depth) & (roi_depth>0) & (roi_depth<=depth_max)
        mask = mask_color & depth_ok
    else:
        mask = mask_color
    ys, xs = np.where(mask)
    if xs.size < min_pixels: return None, 0
    bx0 = int(x0 + xs.min()); bx1 = int(x0 + xs.max())
    by0 = int(y0 + ys.min()); by1 = int(y0 + ys.max())
    return (bx0, by0, bx1, by1), int(xs.size)

# --- mapping & filtering (exactly as in your detection script) ---

def load_actor_map(csv_path):
    mapping = {}
    try:
        with open(csv_path, "r", newline="") as f:
            sniffer = csv.Sniffer()
            sample = f.read(1024); f.seek(0)
            has_header = False
            try: has_header = sniffer.has_header(sample)
            except Exception: pass
            reader = csv.reader(f)
            if has_header: next(reader, None)
            for row in reader:
                if not row or len(row)<2: continue
                actor_label = row[0].strip()
                idname      = row[1].strip()
                if idname: mapping[idname] = actor_label
    except FileNotFoundError:
        print(f"[WARN] CSV not found: {csv_path} (continuing without labels)")
    except Exception as e:
        print(f"[WARN] Error reading CSV '{csv_path}': {e} (continuing without labels)")
    return mapping

def label_has_keyword(name_or_label, keywords):
    if not name_or_label: return False
    s_raw = str(name_or_label)
    s_cc  = re.sub(r'(?<=[a-z])(?=[A-Z0-9])', ' ', s_raw)
    s_cc  = re.sub(r'(?<=[A-Z])(?=[0-9])', ' ', s_cc)
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

def infer_object_type_from_label(label_or_id):
    s = (label_or_id or "").lower()
    if ("human" in s) or ("person" in s) or ("pedestrian" in s) or ("splinehuman" in s):
        return "human"
    if any(k in s for k in ("car","truck","sedan","suv","vehicle","van","bus","sportscar","policecar","pickup","taxi")):
        return "car"
    return "car"

def get_profile_for_idname(idname, obj_type):
    prof = PROFILES.get(obj_type, PROFILES["car"]).copy()
    lname = idname.lower()
    for substr, over in PROFILE_OVERRIDES_BY_ID_SUBSTR.items():
        if substr in lname:
            prof.update(over)
            break
    if "FWD_OFF_M" not in prof: prof["FWD_OFF_M"] = 0.0
    return prof

def pose_with_offsets(base_pose, z_off_m=0.0, fwd_off_m=0.0):
    R = quat_to_rot_matrix(base_pose.orientation.w_val, base_pose.orientation.x_val,
                           base_pose.orientation.y_val, base_pose.orientation.z_val)
    fwd = R[:,0]
    p = np.array([base_pose.position.x_val, base_pose.position.y_val, base_pose.position.z_val], dtype=float)
    p = p + fwd_off_m * fwd
    p[2] = p[2] + z_off_m
    return airsim.Pose(airsim.Vector3r(p[0],p[1],p[2]), base_pose.orientation)

# ------------------ IMAGE / DEPTH / SEG ------------------

def get_images(client, vehicle_name):
    reqs = [
        airsim.ImageRequest(CAM_NAME, airsim.ImageType.Scene, False, True),
        airsim.ImageRequest(CAM_NAME, airsim.ImageType.DepthPlanar, True, False),
        airsim.ImageRequest(CAM_NAME, airsim.ImageType.Segmentation, False, False),
    ]
    while True:
        imgs = client.simGetImages(reqs, vehicle_name=vehicle_name)
        scene, depth, seg = imgs
        if scene.width>0 and scene.height>0 and \
           (scene.compress and len(scene.image_data_uint8)>0) and \
           depth.pixels_as_float and depth.image_data_float and \
           seg.width>0 and seg.height>0 and len(seg.image_data_uint8)>0:
            data = np.frombuffer(scene.image_data_uint8, np.uint8)
            scene_img = cv2.imdecode(data, cv2.IMREAD_COLOR)
            depth_arr = np.array(depth.image_data_float, np.float32).reshape(depth.height, depth.width)
            seg_arr   = np.frombuffer(seg.image_data_uint8, np.uint8).reshape(seg.height, seg.width, 3)
            seg_bgr   = cv2.cvtColor(seg_arr, cv2.COLOR_RGB2BGR)
            return scene_img, depth_arr, seg_bgr

def get_lidar_points_with_retry(client, vehicle_name, max_retries=3, backoff_s=0.001):
    for _ in range(max_retries):
        ld = client.getLidarData(lidar_name=LIDAR_NAME, vehicle_name=vehicle_name)
        if ld.point_cloud:
            pts = np.array(ld.point_cloud, np.float32).reshape(-1, 3)
            if pts.size: return pts
        time.sleep(backoff_s)
    return None

def save_lidar_bin(path, pts_xyz):
    arr = np.concatenate([pts_xyz, np.ones((pts_xyz.shape[0],1), np.float32)], axis=1).astype(np.float32)
    arr.tofile(path)

# ------------------ CALIBRATION ------------------

def build_sensor_calibrations(settings, vehicle_name):
    vehicle_cfg = settings["Vehicles"].get(vehicle_name)
    if vehicle_cfg is None:
        raise RuntimeError(f"Vehicle {vehicle_name} not in settings.json")
    cam_cfg = vehicle_cfg["Cameras"][CAM_NAME]

    capture = None
    for cap in cam_cfg.get("CaptureSettings", []):
        if cap.get("ImageType", -1) == 0:
            capture = cap; break
    if capture is None:
        raise RuntimeError(f"No scene capture for camera {CAM_NAME} on {vehicle_name}")

    width  = capture["Width"]
    height = capture["Height"]
    fov_deg= capture["FOV_Degrees"]
    K      = compute_intrinsics_from_horizontal_fov(fov_deg, width, height)
    P      = projection_from_K(K)

    cam_X, cam_Y, cam_Z = cam_cfg.get("X",0.0), cam_cfg.get("Y",0.0), cam_cfg.get("Z",0.0)
    cam_Pitch,cam_Roll,cam_Yaw = cam_cfg.get("Pitch",0.0), cam_cfg.get("Roll",0.0), cam_cfg.get("Yaw",0.0)
    R_cam = euler_to_rot_matrix(cam_Yaw, cam_Pitch, cam_Roll)
    T_cam_to_vehicle = build_homogeneous(R_cam, np.array([cam_X, cam_Y, cam_Z], dtype=float))

    lidar_cfg = vehicle_cfg["Sensors"][LIDAR_NAME]
    lidar_X, lidar_Y, lidar_Z = lidar_cfg.get("X",0.0), lidar_cfg.get("Y",0.0), lidar_cfg.get("Z",0.0)
    lidar_Roll, lidar_Pitch, lidar_Yaw = lidar_cfg.get("Roll",0.0), lidar_cfg.get("Pitch",0.0), lidar_cfg.get("Yaw",0.0)
    R_lidar = euler_to_rot_matrix(lidar_Yaw, lidar_Pitch, lidar_Roll)
    T_lidar_to_vehicle = build_homogeneous(R_lidar, np.array([lidar_X, lidar_Y, lidar_Z], dtype=float))

    return {"K":K, "P":P, "image_size":(width,height),
            "T_cam_to_vehicle":T_cam_to_vehicle,
            "T_lidar_to_vehicle":T_lidar_to_vehicle}

# ------------------ MAIN ------------------

def main():
    print("[ENTRY] Starting main()")
    settings = load_settings(SETTINGS_JSON_PATH)
    print(f"[OK] loaded settings.json from {SETTINGS_JSON_PATH}")

    veh_calib = build_sensor_calibrations(settings, SIDE_VEH[0])
    inf_calib = build_sensor_calibrations(settings, SIDE_INF[0])
    print("[OK] built calibrations")

    # set AirSim detection radius filters (just to pre-trim candidate lists)
    def set_detection_filters(client, vehicle_name, radius_cm, mesh_names=None):
        client.simClearDetectionMeshNames(CAM_NAME, airsim.ImageType.Scene, vehicle_name=vehicle_name)
        client.simSetDetectionFilterRadius(CAM_NAME, airsim.ImageType.Scene, radius_cm, vehicle_name=vehicle_name)
        if not mesh_names:
            client.simAddDetectionFilterMeshName(CAM_NAME, airsim.ImageType.Scene, "*", vehicle_name=vehicle_name)
        else:
            for pattern in mesh_names:
                client.simAddDetectionFilterMeshName(CAM_NAME, airsim.ImageType.Scene, pattern, vehicle_name=vehicle_name)

    set_detection_filters(car_client,   SIDE_VEH[0], VEHICLE_DET_RADIUS_CM, mesh_names=MESH_FILTERS["veh"])
    set_detection_filters(drone_client, SIDE_INF[0], INF_DET_RADIUS_CM,     mesh_names=MESH_FILTERS["inf"])
    print("[OK] detection filters set")

    # pre-load label mapping
    id_to_label = load_actor_map(CSV_PATH)

    # Dump static calibration JSONs (virtual leveled lidar for yaw)
    T_world_veh_init = get_vehicle_world_transform(car_client, SIDE_VEH[0], is_drone=False)
    T_world_inf_init = get_vehicle_world_transform(drone_client, SIDE_INF[0], is_drone=True)

    def write_calib(side_name, T_world_vehicle, calib, client, vehicle_name):
        T_world_lidar_raw = T_world_vehicle @ calib["T_lidar_to_vehicle"]
        R_raw = T_world_lidar_raw[:3,:3]; t_raw = T_world_lidar_raw[:3,3]
        R_virtual = F_VL @ R_raw @ F_VL
        yaw = extract_yaw_from_virtual_lidar_rotation(R_virtual)
        R_level = np.array([[ math.cos(yaw),-math.sin(yaw),0],
                            [ math.sin(yaw), math.cos(yaw),0],
                            [ 0,0,1]], dtype=float)
        T_world_vlidar = build_homogeneous(R_level, t_raw)
        T_world_cam = T_world_vehicle @ calib["T_cam_to_vehicle"]
        cam2vlidar_virtual = invert_homogeneous(T_world_cam) @ T_world_vlidar
        out = {
            "cam_intrinsic": calib["K"].tolist(),
            "cam2vlidar":    cam2vlidar_virtual.tolist(),
            "vlidar2world":  T_world_vlidar.tolist(),
        }
        json.dump(out, open(os.path.join(PATHS[side_name]["calib"], "calib.json"), "w"), indent=2)

    write_calib("veh", T_world_veh_init, veh_calib, car_client,   SIDE_VEH[0])
    write_calib("inf", T_world_inf_init, inf_calib, drone_client, SIDE_INF[0])

    # --- MAIN LOOP ---
    num_ticks = int(DURATION_S * BASE_HZ)
    print(f"[COLLECT] Collecting {num_ticks} ticks @ {BASE_HZ}Hz (DT={DT:.3f}s)")

    veh_ts_list, inf_ts_list, frame_pairs = [], [], []

    # current frames (used for detection on the synchronized tick)
    curr_scene_veh = curr_depth_veh = curr_seg_veh = None
    curr_scene_inf = curr_depth_inf = curr_seg_inf = None

    for tick in range(1, num_ticks+1):
        try:
            drone_client.simContinueForTime(DT)
        except Exception as e:
            print(f"[ERROR] sim step failed at tick {tick}: {e}")
            continue

        t_ms = int(round(tick * DT * 1000.0))
        if tick % 100 == 0:
            print(f"[COLLECT] tick {tick}/{num_ticks}")

        # capture vehicle
        if tick % CAM_EVERY == 0:
            img, dep, seg = get_images(car_client, SIDE_VEH[0])
            cv2.imwrite(os.path.join(PATHS["veh"]["img"], f"{t_ms}{IMG_EXT}"), img)
            np.save(os.path.join(PATHS["veh"]["depth"], f"{t_ms}.npy"), dep)
            cv2.imwrite(os.path.join(PATHS["veh"]["seg"], f"{t_ms}{IMG_EXT}"), seg)
            curr_scene_veh, curr_depth_veh, curr_seg_veh = img, dep, seg
        veh_got_lidar = False
        if tick % LIDAR_EVERY == 0:
            pts = get_lidar_points_with_retry(car_client, SIDE_VEH[0])
            if pts is not None and pts.size:
                save_lidar_bin(os.path.join(PATHS["veh"]["lidar"], f"{t_ms}.bin"), pts)
                veh_ts_list.append(t_ms)
                veh_got_lidar = True

        # capture infra
        if tick % CAM_EVERY == 0:
            img, dep, seg = get_images(drone_client, SIDE_INF[0])
            cv2.imwrite(os.path.join(PATHS["inf"]["img"], f"{t_ms}{IMG_EXT}"), img)
            np.save(os.path.join(PATHS["inf"]["depth"], f"{t_ms}.npy"), dep)
            cv2.imwrite(os.path.join(PATHS["inf"]["seg"], f"{t_ms}{IMG_EXT}"), seg)
            curr_scene_inf, curr_depth_inf, curr_seg_inf = img, dep, seg
        inf_got_lidar = False
        if tick % LIDAR_EVERY == 0:
            pts = get_lidar_points_with_retry(drone_client, SIDE_INF[0])
            if pts is not None and pts.size:
                save_lidar_bin(os.path.join(PATHS["inf"]["lidar"], f"{t_ms}.bin"), pts)
                inf_ts_list.append(t_ms)
                inf_got_lidar = True

        # synchronized tick → generate labels
        if veh_got_lidar and inf_got_lidar:
            frame_pairs.append((t_ms, t_ms))

            # camera poses (same paused step)
            cam_info_veh = car_client.simGetCameraInfo(CAM_NAME, SIDE_VEH[0])
            cam_info_inf = drone_client.simGetCameraInfo(CAM_NAME, SIDE_INF[0])
            cam_pose_veh = cam_info_veh.pose
            cam_pose_inf = cam_info_inf.pose
            P_veh, P_inf = veh_calib["P"], inf_calib["P"]
            W_veh, H_veh = veh_calib["image_size"]
            W_inf, H_inf = inf_calib["image_size"]

            # list candidates via AirSim detections (trim), then filter by KEYWORDS (exact method)
            veh_dets = car_client.simGetDetections(CAM_NAME, airsim.ImageType.Scene, vehicle_name=SIDE_VEH[0]) or []
            inf_dets = drone_client.simGetDetections(CAM_NAME, airsim.ImageType.Scene, vehicle_name=SIDE_INF[0]) or []

            # --- VEHICLE SIDE ---
            veh_labels = []
            for d in veh_dets:
                # keyword filter (label mapping or idname)
                pretty_label = id_to_label.get(d.name, "")
                if not (label_has_keyword(pretty_label, KEYWORDS) or label_has_keyword(d.name, KEYWORDS)):
                    continue

                text_for_type = pretty_label if pretty_label else d.name
                obj_type = infer_object_type_from_label(text_for_type)
                profile  = get_profile_for_idname(d.name, obj_type)
                L,W,Hh   = profile["L"], profile["W"], profile["H"]

                pose_world = get_object_world_pose(car_client, d.name)
                if pose_world is None:
                    continue

                # amodal 2D bbox from projected 3D cuboid
                bbox, has_any_valid, corners_w, adj_pose, (L,W,Hh), depth_fwd, valid \
                    = amodal_bbox_for_actor_with_dims(pose_world, cam_pose_veh, P_veh, W_veh, H_veh,
                                                      L, W, Hh, profile["Z"], profile.get("FWD_OFF_M", 0.0))
                if not has_any_valid or bbox is None:
                    continue

                x0,y0,x1,y1 = bbox
                # ROI → depth-clip → back-project → dominant color inside 3D box
                seg_roi   = curr_seg_veh[y0:y1+1, x0:x1+1, :]
                depth_roi = curr_depth_veh[y0:y1+1, x0:x1+1]
                if DEPTH_CLIP_ENABLE:
                    vd_mask = np.isfinite(depth_roi) & (depth_roi>0) & (depth_roi<=DEPTH_CLIP_MAX_M)
                    seg_roi_clipped = np.zeros_like(seg_roi)
                    seg_roi_clipped[vd_mask] = seg_roi[vd_mask]
                else:
                    seg_roi_clipped = seg_roi

                world_pts, colors = roi_points_to_world_pointcloud_P(seg_roi_clipped, depth_roi,
                                                                     x0, y0, W_veh, H_veh, P_veh, cam_pose_veh,
                                                                     stride=ROI_POINT_STRIDE)
                if world_pts is not None and world_pts.shape[0] > 0:
                    top_colors, n_inside = dominant_colors_in_box(world_pts, colors, adj_pose, L, W, Hh, top_k=3)
                    if n_inside == 0:
                        # occluded → drop
                        continue
                    best_rgb, _, _ = top_colors[0]
                    # search rect: amodal ± margin
                    sr = (max(0, x0-REFIT_SEARCH_MARGIN_PX),
                          max(0, y0-REFIT_SEARCH_MARGIN_PX),
                          min(W_veh-1, x1+REFIT_SEARCH_MARGIN_PX),
                          min(H_veh-1, y1+REFIT_SEARCH_MARGIN_PX))
                    tight, pix = tight_box_for_color(curr_seg_veh, curr_depth_veh, best_rgb, sr,
                                                     use_depth=REFIT_USE_DEPTH_CLIP_FOR_TIGHT_BOX and DEPTH_CLIP_ENABLE,
                                                     depth_max=DEPTH_CLIP_MAX_M,
                                                     min_pixels=REFIT_MIN_PIXELS)
                    if tight is None:
                        continue
                    tx0,ty0,tx1,ty1 = tight
                    if (tx1-tx0)*(ty1-ty0) < MIN_BBOX_AREA or min(tx1-tx0, ty1-ty0) < min(MIN_BBOX_WIDTH, MIN_BBOX_HEIGHT):
                        continue
                    # final 2D box
                    bx0,by0,bx1,by1 = tx0,ty0,tx1,ty1
                else:
                    # fallback—use amodal if ROI had no valid points at all
                    bx0,by0,bx1,by1 = x0,y0,x1,y1

                # 3D location in camera frame (AirSim camera: X forward, Y right, Z down)
                R_cam = quat_to_rot_matrix(cam_pose_veh.orientation.w_val, cam_pose_veh.orientation.x_val,
                                           cam_pose_veh.orientation.y_val, cam_pose_veh.orientation.z_val)
                cam_p = np.array([cam_pose_veh.position.x_val, cam_pose_veh.position.y_val, cam_pose_veh.position.z_val], dtype=float)
                obj_c = np.array([adj_pose.position.x_val, adj_pose.position.y_val, adj_pose.position.z_val], dtype=float)
                p_cam = R_cam.T @ (obj_c - cam_p)

                # rotation (yaw proxy in camera frame)
                R_obj = quat_to_rot_matrix(adj_pose.orientation.w_val, adj_pose.orientation.x_val,
                                           adj_pose.orientation.y_val, adj_pose.orientation.z_val)
                heading_cam = R_cam.T @ R_obj @ np.array([1,0,0], dtype=float)
                rot_yaw = math.atan2(heading_cam[1], heading_cam[0])

                veh_labels.append({
                    "type": "Pedestrian" if obj_type=="human" else "Car",
                    "occluded_state": 0,
                    "truncated_state": 0,
                    "alpha": float(rot_yaw),
                    "2d_box": {"xmin": float(bx0), "ymin": float(by0), "xmax": float(bx1), "ymax": float(by1)},
                    "3d_dimensions": {"h": float(Hh), "w": float(W), "l": float(L)},
                    "3d_location": {"x": float(p_cam[0]), "y": float(p_cam[1]), "z": float(p_cam[2])},
                    "rotation": float(rot_yaw)
                })

            # --- INFRA SIDE (mirror of above) ---
            inf_labels = []
            for d in inf_dets:
                pretty_label = id_to_label.get(d.name, "")
                if not (label_has_keyword(pretty_label, KEYWORDS) or label_has_keyword(d.name, KEYWORDS)):
                    continue

                text_for_type = pretty_label if pretty_label else d.name
                obj_type = infer_object_type_from_label(text_for_type)
                profile  = get_profile_for_idname(d.name, obj_type)
                L,W,Hh   = profile["L"], profile["W"], profile["H"]

                pose_world = get_object_world_pose(drone_client, d.name)
                if pose_world is None:
                    continue

                bbox, has_any_valid, corners_w, adj_pose, (L,W,Hh), depth_fwd, valid \
                    = amodal_bbox_for_actor_with_dims(pose_world, cam_pose_inf, P_inf, W_inf, H_inf,
                                                      L, W, Hh, profile["Z"], profile.get("FWD_OFF_M", 0.0))
                if not has_any_valid or bbox is None:
                    continue

                x0,y0,x1,y1 = bbox
                seg_roi   = curr_seg_inf[y0:y1+1, x0:x1+1, :]
                depth_roi = curr_depth_inf[y0:y1+1, x0:x1+1]
                if DEPTH_CLIP_ENABLE:
                    vd_mask = np.isfinite(depth_roi) & (depth_roi>0) & (depth_roi<=DEPTH_CLIP_MAX_M)
                    seg_roi_clipped = np.zeros_like(seg_roi)
                    seg_roi_clipped[vd_mask] = seg_roi[vd_mask]
                else:
                    seg_roi_clipped = seg_roi

                world_pts, colors = roi_points_to_world_pointcloud_P(seg_roi_clipped, depth_roi,
                                                                     x0, y0, W_inf, H_inf, P_inf, cam_pose_inf,
                                                                     stride=ROI_POINT_STRIDE)
                if world_pts is not None and world_pts.shape[0] > 0:
                    top_colors, n_inside = dominant_colors_in_box(world_pts, colors, adj_pose, L, W, Hh, top_k=3)
                    if n_inside == 0:
                        continue
                    best_rgb, _, _ = top_colors[0]
                    sr = (max(0, x0-REFIT_SEARCH_MARGIN_PX),
                          max(0, y0-REFIT_SEARCH_MARGIN_PX),
                          min(W_inf-1, x1+REFIT_SEARCH_MARGIN_PX),
                          min(H_inf-1, y1+REFIT_SEARCH_MARGIN_PX))
                    tight, pix = tight_box_for_color(curr_seg_inf, curr_depth_inf, best_rgb, sr,
                                                     use_depth=REFIT_USE_DEPTH_CLIP_FOR_TIGHT_BOX and DEPTH_CLIP_ENABLE,
                                                     depth_max=DEPTH_CLIP_MAX_M,
                                                     min_pixels=REFIT_MIN_PIXELS)
                    if tight is None:
                        continue
                    tx0,ty0,tx1,ty1 = tight
                    if (tx1-tx0)*(ty1-ty0) < MIN_BBOX_AREA or min(tx1-tx0, ty1-ty0) < min(MIN_BBOX_WIDTH, MIN_BBOX_HEIGHT):
                        continue
                    bx0,by0,bx1,by1 = tx0,ty0,tx1,ty1
                else:
                    bx0,by0,bx1,by1 = x0,y0,x1,y1

                R_cam = quat_to_rot_matrix(cam_pose_inf.orientation.w_val, cam_pose_inf.orientation.x_val,
                                           cam_pose_inf.orientation.y_val, cam_pose_inf.orientation.z_val)
                cam_p = np.array([cam_pose_inf.position.x_val, cam_pose_inf.position.y_val, cam_pose_inf.position.z_val], dtype=float)
                obj_c = np.array([adj_pose.position.x_val, adj_pose.position.y_val, adj_pose.position.z_val], dtype=float)
                p_cam = R_cam.T @ (obj_c - cam_p)

                R_obj = quat_to_rot_matrix(adj_pose.orientation.w_val, adj_pose.orientation.x_val,
                                           adj_pose.orientation.y_val, adj_pose.orientation.z_val)
                heading_cam = R_cam.T @ R_obj @ np.array([1,0,0], dtype=float)
                rot_yaw = math.atan2(heading_cam[1], heading_cam[0])

                inf_labels.append({
                    "type": "Pedestrian" if obj_type=="human" else "Car",
                    "occluded_state": 0,
                    "truncated_state": 0,
                    "alpha": float(rot_yaw),
                    "2d_box": {"xmin": float(bx0), "ymin": float(by0), "xmax": float(bx1), "ymax": float(by1)},
                    "3d_dimensions": {"h": float(Hh), "w": float(W), "l": float(L)},
                    "3d_location": {"x": float(p_cam[0]), "y": float(p_cam[1]), "z": float(p_cam[2])},
                    "rotation": float(rot_yaw)
                })

            # write labels (DAIR-V2X KITTI-style JSON) for this timestamp
            with open(os.path.join(PATHS["label"]["veh"], f"{t_ms:06d}.json"), "w") as f:
                json.dump(veh_labels, f)
            with open(os.path.join(PATHS["label"]["inf"], f"{t_ms:06d}.json"), "w") as f:
                json.dump(inf_labels, f)

            # mirror to per-side “kitti_label/” dirs (same content), keyed by image timestamp
            with open(os.path.join(PATHS["veh"]["kitti_label"], f"{t_ms:06d}.json"), "w") as f:
                json.dump(veh_labels, f)
            with open(os.path.join(PATHS["inf"]["kitti_label"], f"{t_ms:06d}.json"), "w") as f:
                json.dump(inf_labels, f)

    # Write timestamp lists
    with open(os.path.join(PATHS["veh"]["ts"], "timestamp.txt"), "w") as f:
        for t in veh_ts_list: f.write(f"{t}\n")
    with open(os.path.join(PATHS["inf"]["ts"], "timestamp.txt"), "w") as f:
        for t in inf_ts_list: f.write(f"{t}\n")

    print("[DONE] Collection complete.")

if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
