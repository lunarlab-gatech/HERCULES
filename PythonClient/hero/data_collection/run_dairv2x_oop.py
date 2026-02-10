#!/usr/bin/env python3
import os, json, math, time, traceback
import numpy as np
import cv2
import setup_path
import hercules as airsim

from Hercules2D3DDetector import Hercules2D3DDetector as H

# ===================== CONFIG =====================
DRONE_PORT = 41451
HUSKY_PORT = 41452

SIDE_INF = ["Drone1"]   # UAV / infrastructure
SIDE_VEH = ["Husky1"]   # UGV / vehicle

CAM_NAME   = H.CAMERA_NAME         # use your class default ("front_center")
LIDAR_NAME = "LidarSensor1"

# DAIR-V2X-C defaults (10 Hz cam/lidar)
CAM_RATE   = 10
LIDAR_RATE = 10
DURATION_S = 300.0

OUT_ROOT   = "/home/sgarimella34/multi-robot-coordination/collaborative-perception-BEVP/datasets/dair_v2x_synth/"

IMG_EXT    = ".png"
BASE_HZ    = CAM_RATE
DT         = 1.0 / BASE_HZ
CAM_EVERY  = BASE_HZ // CAM_RATE
LIDAR_EVERY = BASE_HZ // LIDAR_RATE

# ------------------ OUTPUT LAYOUT ------------------
def dair_paths(root):
    return {
        "inf": {
            "img":         f"{root}/cooperative/infrastructure-side/image",
            "depth":       f"{root}/cooperative/infrastructure-side/depth",
            "seg":         f"{root}/cooperative/infrastructure-side/seg",
            "lidar":       f"{root}/cooperative/infrastructure-side/lidar",
            "calib":       f"{root}/cooperative/infrastructure-side/calib",
            "ts":          f"{root}/cooperative/infrastructure-side/timestamp",
            "kitti_label": f"{root}/cooperative/infrastructure-side/kitti_label",
            "kitti_label_pp": f"{root}/cooperative/infrastructure-side/kitti_label_pp"
        },
        "veh": {
            "img":         f"{root}/cooperative/vehicle-side/image",
            "depth":       f"{root}/cooperative/vehicle-side/depth",
            "seg":         f"{root}/cooperative/vehicle-side/seg",
            "lidar":       f"{root}/cooperative/vehicle-side/lidar",
            "calib":       f"{root}/cooperative/vehicle-side/calib",
            "ts":          f"{root}/cooperative/vehicle-side/timestamp",
            "kitti_label": f"{root}/cooperative/vehicle-side/kitti_label",
            "kitti_label_pp": f"{root}/cooperative/vehicle-side/kitti_label_pp"
        },
        "label": {
            "veh":         f"{root}/cooperative/label/vehicle",
            "inf":         f"{root}/cooperative/label/infrastructure",
            "cooperative": f"{root}/cooperative/label/cooperative"
        }
    }

PATHS = dair_paths(OUT_ROOT)
for group in PATHS.values():
    for p in group.values():
        os.makedirs(p, exist_ok=True)

# ------------------ HELPERS ------------------
def _deg2rad(d): 
    return d * math.pi / 180.0

def _R_from_rpy_deg(roll_deg, pitch_deg, yaw_deg):
    """
    AirSim convention (NED, right-handed):
      - X: forward, Y: right, Z: down
      - Roll about +X, Pitch about +Y, Yaw about +Z
    We use column vectors; rotation composition is Rz(yaw) * Ry(pitch) * Rx(roll).
    """
    r, p, y = map(_deg2rad, (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)

    Rx = np.array([[1, 0, 0],
                   [0, cr, -sr],
                   [0, sr,  cr]], dtype=float)
    Ry = np.array([[ cp, 0, sp],
                   [  0, 1,  0],
                   [-sp, 0, cp]], dtype=float)
    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [ 0,   0, 1]], dtype=float)
    return Rz @ Ry @ Rx

def _T_from_xyzrpy(x, y, z, roll_deg, pitch_deg, yaw_deg):
    """Vehicle->Sensor transform from settings: translation in meters; angles in deg."""
    T = np.eye(4, dtype=float)
    T[:3,:3] = _R_from_rpy_deg(roll_deg, pitch_deg, yaw_deg)
    T[:3, 3] = np.array([x, y, z], dtype=float)
    return T

def _load_settings(path=None):
    """
    Loads AirSim settings.json. If path is None, uses default:
      Windows:  %USERPROFILE%/Documents/AirSim/settings.json
      Linux:    ~/Documents/AirSim/settings.json
    """
    if path is None:
        home = os.path.expanduser("~")
        path = os.path.join(home, "Documents", "AirSim", "settings.json")
    with open(path, "r") as f:
        return json.load(f)

def extrinsics_from_settings(vehicle_name, cam_name, lidar_name, settings_path=None):
    """
    Returns T_cam_lidar (4x4) built purely from settings.json for ONE vehicle.
      vehicle_name: e.g. "Drone1" or "Husky1"
      cam_name:     e.g. "front_center"
      lidar_name:   e.g. "LidarSensor1"
    """
    js = _load_settings(settings_path)
    V = js.get("Vehicles", {})
    if vehicle_name not in V:
        raise KeyError(f"Vehicle '{vehicle_name}' not found in settings.json")

    v = V[vehicle_name]

    # Camera pose (relative to VEHICLE body)
    cams = v.get("Cameras", {})
    if cam_name not in cams:
        raise KeyError(f"Camera '{cam_name}' not found under vehicle '{vehicle_name}'")
    c = cams[cam_name]
    cx, cy, cz = c.get("X", 0.0), c.get("Y", 0.0), c.get("Z", 0.0)
    croll, cpitch, cyaw = c.get("Roll", 0.0), c.get("Pitch", 0.0), c.get("Yaw", 0.0)
    T_v_to_c = _T_from_xyzrpy(cx, cy, cz, croll, cpitch, cyaw)    # ^cT_v? (see below)

    # LiDAR pose (relative to VEHICLE body)
    sens = v.get("Sensors", {})
    if lidar_name not in sens:
        raise KeyError(f"LiDAR '{lidar_name}' not found under vehicle '{vehicle_name}'")
    l = sens[lidar_name]
    lx, ly, lz = l.get("X", 0.0), l.get("Y", 0.0), l.get("Z", 0.0)
    lroll, lpitch, lyaw = l.get("Roll", 0.0), l.get("Pitch", 0.0), l.get("Yaw", 0.0)
    T_v_to_l = _T_from_xyzrpy(lx, ly, lz, lroll, lpitch, lyaw)

    # Important: AirSim sensor poses in settings are SENSOR w.r.t VEHICLE (i.e., vehicle→sensor).
    # We want lidar→camera:
    #   p_cam = T_cam_lidar * p_lidar
    #   T_cam_lidar = (T_v→c)^(-1) * (T_v→l)
    T_cam_lidar = np.linalg.inv(T_v_to_c) @ T_v_to_l
    return T_cam_lidar


# --- Pose helpers (compose vehicle ⟶ sensor into world frame) ---
def _pose_to_T(pose):
    """AirSim Pose -> 4x4 world transform (R|t)."""
    R = H.quaternion_to_rotation_matrix(pose.orientation)
    t = np.array([pose.position.x_val, pose.position.y_val, pose.position.z_val], dtype=float)
    T = np.eye(4, dtype=float)
    T[:3,:3] = R
    T[:3, 3] = t
    return T


def _compose_world_from_vehicle_and_sensor(veh_pose, sensor_pose):
    """
    Return (R_ws, t_ws) for world->sensor by composing
    world->vehicle with vehicle->sensor.
    Works whether sensor_pose is given in vehicle or world frame.
    """
    R_wv = H.quaternion_to_rotation_matrix(veh_pose.orientation)
    t_wv = np.array([veh_pose.position.x_val, veh_pose.position.y_val, veh_pose.position.z_val], dtype=float)
    R_vs = H.quaternion_to_rotation_matrix(sensor_pose.orientation)
    t_vs = np.array([sensor_pose.position.x_val, sensor_pose.position.y_val, sensor_pose.position.z_val], dtype=float)
    R_ws = R_wv @ R_vs
    t_ws = t_wv + R_wv @ t_vs
    return R_ws, t_ws


def get_images(client, vehicle_name):
    """Return (BGR uint8), (depth_m float32), (seg BGR uint8) with matching WxH."""
    reqs = [
        airsim.ImageRequest(CAM_NAME, airsim.ImageType.Scene,            False, False),
        airsim.ImageRequest(CAM_NAME, airsim.ImageType.DepthPerspective, True,  False),  # <-- was DepthPlanar
        airsim.ImageRequest(CAM_NAME, airsim.ImageType.Segmentation,     False, False),
    ]
    while True:
        imgs = client.simGetImages(reqs, vehicle_name=vehicle_name)
        if len(imgs) != 3:
            continue
        scene, depth, seg = imgs
        if scene.width <= 0 or scene.height <= 0:
            continue

        # RGB
        rgb = np.frombuffer(scene.image_data_uint8, dtype=np.uint8).reshape(scene.height, scene.width, 3)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        H, W = bgr.shape[:2]

        # Depth (Perspective = true range)
        depth_m = np.array(depth.image_data_float, dtype=np.float32).reshape(depth.height, depth.width)
        if depth_m.shape[:2] != (H, W):
            depth_m = cv2.resize(depth_m, (W, H), interpolation=cv2.INTER_NEAREST)

        # Segmentation
        seg_rgb = np.frombuffer(seg.image_data_uint8, dtype=np.uint8).reshape(seg.height, seg.width, 3)
        seg_bgr = cv2.cvtColor(seg_rgb, cv2.COLOR_RGB2BGR)
        if seg_bgr.shape[:2] != (H, W):
            seg_bgr = cv2.resize(seg_bgr, (W, H), interpolation=cv2.INTER_NEAREST)

        return bgr, depth_m, seg_bgr


def get_lidar_points(client, vehicle_name):
    ld = client.getLidarData(lidar_name=LIDAR_NAME, vehicle_name=vehicle_name)
    if ld and ld.point_cloud:
        pts = np.array(ld.point_cloud, dtype=np.float32).reshape(-1, 3)
        if pts.size:
            return pts
    return None

def save_lidar_bin(path, xyz):
    """
    Save KITTI .bin with x,y,z,1.0 float32 in **DAIR-V2X Virtual LiDAR** coords.
    AirSim LiDAR returns points in a body/sensor frame where:
      X forward, Y right, Z down. DAIR requires: X forward, Y left, Z up.
    Convert by flipping Y and Z.
    """
    if xyz is None or not len(xyz):
        return
    pts = xyz.astype(np.float32).copy()
    # AirSim (x,+y right,+z down) -> DAIR (x,+y left,+z up)
    pts[:, 1] = -pts[:, 1]
    pts[:, 2] = -pts[:, 2]
    N = pts.shape[0]
    arr = np.hstack([pts, np.ones((N, 1), dtype=np.float32)])
    arr.astype(np.float32).tofile(path)


def build_calib_json(client, side_name):
    img, depth, seg = get_images(client, side_name)
    h, w = img.shape[:2]

    info = client.simGetCameraInfo(CAM_NAME, vehicle_name=side_name)
    # Prefer intrinsics from AirSim projection matrix when it looks valid
    P = np.array(info.proj_mat.matrix, dtype=np.float64).reshape((4,4))
    if np.isfinite(P).all() and not np.allclose(P, 0):
        K = P[:3, :3].copy()
    else:
        # Fallback: derive from horizontal FOV
        K, _vfov = H.compute_intrinsics_from_horizontal_fov(info.fov, w, h)
        P = np.eye(4, dtype=float); P[:3,:3] = K
    # --- Build T_cam_lidar (lidar -> camera) ---
    T_cam_lidar = None

    # 1) # 1) Try runtime (preferred) – compose via vehicle pose so both camera & lidar are in world
    try:
        veh_pose = client.simGetVehiclePose(vehicle_name=side_name)
        # camera: compose world←vehicle←camera_local
        R_wc, t_wc = _compose_world_from_vehicle_and_sensor(veh_pose, info.pose)
        T_wc = np.eye(4, dtype=float); T_wc[:3,:3] = R_wc; T_wc[:3,3] = t_wc

        ld = client.getLidarData(lidar_name=LIDAR_NAME, vehicle_name=side_name)
        if hasattr(ld, "pose") and ld.pose is not None:
            # lidar: compose world←vehicle←lidar_local
            R_wl, t_wl = _compose_world_from_vehicle_and_sensor(veh_pose, ld.pose)
            T_wl = np.eye(4, dtype=float); T_wl[:3,:3] = R_wl; T_wl[:3,3] = t_wl

            T_cw = np.linalg.inv(T_wc)
            T_cam_lidar = T_cw @ T_wl  # lidar→camera

    except Exception:
        T_cam_lidar = None

    # 2) Fallback to settings.json (static)
    if T_cam_lidar is None:
        try:
            # If vehicle names differ between sides, just pass that name (e.g., "Drone1" / "Husky1")
            T_cam_lidar = extrinsics_from_settings(side_name, CAM_NAME, LIDAR_NAME, settings_path=None)
        except Exception:
            T_cam_lidar = None  # last resort

    # IMPORTANT: your .bin is saved in DAIR Virtual LiDAR (x fwd, y left, z up),
    # but AirSim extrinsics above are in LiDAR's native NED (x fwd, y right, z down).
    # Convert the lidar *basis* on the right by post-multiplying with diag(1,-1,-1).
    if T_cam_lidar is not None:
        M4 = np.eye(4, dtype=float)
        M4[1,1] = -1.0
        M4[2,2] = -1.0
        # Now T_cam_lidar maps *Virtual LiDAR* -> camera, matching your saved .bin basis.
        T_cam_lidar = T_cam_lidar @ M4

    out = {
        "K": K, "P": P, "image_size": (w, h),
        "camera_name": CAM_NAME, "lidar_name": LIDAR_NAME
    }
    if T_cam_lidar is not None:
        out["T_cam_lidar"] = T_cam_lidar.tolist()
    return out

def write_calib_json(path, calib):
    out = {
        "K": calib["K"].tolist(),
        "P": calib["P"].tolist(),
        "image_size": list(map(int, calib["image_size"])),
        "camera_name": CAM_NAME,
        "lidar_name":  LIDAR_NAME,
    }
    # Preserve lidar→camera extrinsics when available (needed by validator & training)
    if "T_cam_lidar" in calib and calib["T_cam_lidar"] is not None:
        T = calib["T_cam_lidar"]
        out["T_cam_lidar"] = T.tolist() if hasattr(T, "tolist") else T
    with open(path, "w") as f:
        json.dump(out, f, indent=2)


def kitti_json_from_result(res, cam_pose, P, img_size):
    """
    Build a DAIR-V2X/KITTI-style entry from one processed target.
    Prefer the tight 2D box; fall back to amodal; if neither present, rebuild
    from world cuboid corners using H.project_world_points_to_image (AirSim 4x4 P).
    """
    if not res or not res.get("found", False):
        return None

    # unpack width/height without shadowing the class alias H
    img_w, img_h = img_size

    # 2D box preference: tight -> amodal -> rebuild from corners
    box = res.get("tight_bbox_xyxy") or res.get("amodal_bbox_xyxy")
    if box is None:
        corners_w = res.get("corners_w")
        if corners_w is None:
            return None

        # project world cuboid corners using your class projector (handles AirSim P)
        pts2d, depth_forward, valid = H.project_world_points_to_image(
            np.asarray(corners_w, dtype=float), cam_pose, P, img_w, img_h
        )
        if pts2d is None or valid is None:
            return None

        u, v = pts2d[:, 0], pts2d[:, 1]
        in_bounds = (u >= 0) & (u < img_w) & (v >= 0) & (v < img_h)
        use = valid & in_bounds

        # if nothing is simultaneously valid+in-bounds, try valid-only then clamp to image
        if not np.any(use):
            use = valid
            if not np.any(use):
                return None

        x0 = int(np.floor(np.nanmin(u[use])))
        y0 = int(np.floor(np.nanmin(v[use])))
        x1 = int(np.ceil (np.nanmax(u[use])))
        y1 = int(np.ceil (np.nanmax(v[use])))

        # clamp to image bounds
        x0 = max(0, min(img_w - 1, x0))
        y0 = max(0, min(img_h - 1, y0))
        x1 = max(0, min(img_w - 1, x1))
        y1 = max(0, min(img_h - 1, y1))
        if not (x1 > x0 and y1 > y0):
            return None

        box = (x0, y0, x1, y1)

    bx0, by0, bx1, by1 = map(int, box)

    # 3D dims from your class
    Hh = float(res["H"])
    Wd = float(res["W"])
    Ld = float(res["L"])

    # Choose pose (prefer adjusted_pose; fall back to actor_pose if needed)
    adj = res.get("adjusted_pose") or res.get("actor_pose")
    if adj is None:
        return None

    # camera-frame location & yaw (AirSim: X fwd, Y right, Z down)
    R_cam = H.quaternion_to_rotation_matrix(cam_pose.orientation)
    cam_p = np.array([cam_pose.position.x_val,
                      cam_pose.position.y_val,
                      cam_pose.position.z_val], dtype=float)

    obj_c = np.array([adj.position.x_val,
                      adj.position.y_val,
                      adj.position.z_val], dtype=float)
    p_cam = R_cam.T @ (obj_c - cam_p)

    R_obj = H.quaternion_to_rotation_matrix(adj.orientation)
    heading_cam = R_cam.T @ R_obj @ np.array([1, 0, 0], dtype=float)
    # IMPORTANT: With your AirSim camera axes (X fwd, Y right, Z down) and the
    # KITTI-style rotation_y convention, rotation_y corresponds to a rotation
    # around +Z in AirSim camera axes. Use atan2(Y, X).
    rot_yaw = math.atan2(heading_cam[1], heading_cam[0])    
    # label mapping (robust)
    lbl = str(res.get("label", "")).lower()
    if ("human" in lbl) or ("pedestrian" in lbl):
        label_type = "Pedestrian"
    else:
        label_type = "Pedestrian" if H.infer_object_type_from_label(res.get("label","")) == "human" else "Car"

    return {
        "type": label_type,
        "occluded_state": 0,
        "truncated_state": 0,
        "alpha": float(rot_yaw),
        "2d_box": {"xmin": float(bx0), "ymin": float(by0), "xmax": float(bx1), "ymax": float(by1)},
        "3d_dimensions": {"h": float(Hh), "w": float(Wd), "l": float(Ld)},
        "3d_location": {"x": float(p_cam[0]), "y": float(p_cam[1]), "z": float(p_cam[2])},
        "rotation": float(rot_yaw)
    }


# --- PP (LiDAR) label helpers ---

ABS_SETTINGS_PATH = "/home/sgarimella34/Documents/AirSim/settings.json"


def lidar_world_from_settings(client, vehicle_name, lidar_name, settings_path=None):
    """
    Robust world←vehicle←lidar composition using LiDAR mount extrinsics
    from settings.json plus the live vehicle pose. This avoids ambiguity
    in LidarData.pose (which can be vehicle-local or world).
    Returns (R_wl, t_wl).
    """
    js = _load_settings(settings_path)
    v = js.get("Vehicles", {}).get(vehicle_name, {})
    sens = v.get("Sensors", {})
    l = sens.get(lidar_name, {})
    lx, ly, lz = l.get("X", 0.0), l.get("Y", 0.0), l.get("Z", 0.0)
    lroll, lpitch, lyaw = l.get("Roll", 0.0), l.get("Pitch", 0.0), l.get("Yaw", 0.0)
    T_v_to_l = _T_from_xyzrpy(lx, ly, lz, lroll, lpitch, lyaw)   # vehicle→lidar

    veh_pose = client.simGetVehiclePose(vehicle_name=vehicle_name)
    R_wv = H.quaternion_to_rotation_matrix(veh_pose.orientation)
    t_wv = np.array([veh_pose.position.x_val,
                     veh_pose.position.y_val,
                     veh_pose.position.z_val], dtype=float)
    T_wv = np.eye(4, dtype=float); T_wv[:3,:3] = R_wv; T_wv[:3,3] = t_wv

    T_wl = T_wv @ T_v_to_l
    R_wl = T_wl[:3,:3].copy()
    t_wl = T_wl[:3, 3].copy()
    return R_wl, t_wl



def lidar_fov_from_settings(vehicle_name, lidar_name, settings_path=ABS_SETTINGS_PATH):
    js = _load_settings(settings_path)
    try:
        sensor = js["Vehicles"][vehicle_name]["Sensors"][lidar_name]
    except KeyError:
        # Fallback: permissive FOV/range if missing
        return dict(range_m=float("inf"), h_start=-180.0, h_end=180.0, v_lower=-90.0, v_upper=90.0)
    cfg = dict(
        range_m = float(sensor.get("Range", float("inf"))),
        h_start = float(sensor.get("HorizontalFOVStart", -180.0)),
        h_end   = float(sensor.get("HorizontalFOVEnd",   180.0)),
        v_lower = float(sensor.get("VerticalFOVLower",   -90.0)),
        v_upper = float(sensor.get("VerticalFOVUpper",    90.0)),
    )
    # --- Normalize config robustly ---
    # Range: non-positive or non-finite => infinite
    if not math.isfinite(cfg["range_m"]) or cfg["range_m"] <= 0.0:
        cfg["range_m"] = float("inf")
    # Vertical: some configs swap lower/upper; make it increasing and clamp to [-90,90]
    v0, v1 = cfg["v_lower"], cfg["v_upper"]
    if v0 > v1:
        v0, v1 = v1, v0
    cfg["v_lower"] = max(-90.0, min(90.0, v0))
    cfg["v_upper"] = max(-90.0, min(90.0, v1))
    return cfg


def _wrap180(a):
    a = ((a + 180.0) % 360.0) - 180.0
    return a

def _angle_in_interval_deg(a, start, end):
    # Treat nearly-360° spans as always-true
    span = ((end - start) % 360.0 + 360.0) % 360.0
    if span >= 359.999:
        return True
    a = _wrap180(a); start = _wrap180(start); end = _wrap180(end)
    if start <= end:
        return (a >= start) and (a <= end)
    return (a >= start) or (a <= end)


def point_in_lidar_fov_sensor_frame(p_s, h_start, h_end, v_lower, v_upper, range_m):
    x, y, z = float(p_s[0]), float(p_s[1]), float(p_s[2])
    r = math.sqrt(x*x + y*y + z*z)
    if not (math.isfinite(r) and r > 0.0 and r <= range_m):
        return False

    # Match detector: +left/+up angles (AirSim has y RIGHT, z DOWN → flip both)
    yaw = math.degrees(math.atan2(-y, x))
    v   = math.degrees(math.atan2(-z, math.hypot(x, y)))

    # Keep your robust 360°/wrap handling
    delta = (h_end - h_start) % 360.0
    full360 = (abs(delta) < 1e-6)
    if (not full360) and (not _angle_in_interval_deg(yaw, h_start, h_end)):
        return False
    return (v >= v_lower) and (v <= v_upper)


def build_lidar_candidates(client, id_to_label, vehicle_name, R_sw, t_ws, lidar_cfg, strict_fov=True):
    """
    Return list of simple 'res-like' dicts for all objects of interest whose 3D cuboid
    has at least one corner (or center) inside the LiDAR FOV/range (when strict_fov=True).
    When strict_fov=False, skip the angular pre-gate and rely on the later
    'points-inside-cuboid' test to keep only physically supported boxes.
    """
    results = []

    # Types we consider for PP labels (ensure pedestrians aren't dropped)
    TYPES_OK = {"human","pedestrian","person","cyclist","bicycle","motorcycle",
                "car","van","truck","bus"}
    
    try: 
        scene_ids = list(client.simListSceneObjects(".*"))
    except Exception as e:
        print("simListSceneObjects failed:", e)
        scene_ids = []
    idx = 0
    for idname in scene_ids:
        label = id_to_label.get(idname, "")
        # Robust type inference; don't depend on brittle keywords alone.
        obj_type = H.infer_object_type_from_label(label if label else idname)
        obj_type_l = (obj_type or "").lower()
        # Fallback: if we can't infer a type, keep common classes via keyword match.
        if not obj_type_l:
            if H.label_has_keyword(label, H.KEYWORDS) or H.label_has_keyword(idname, H.KEYWORDS):
                obj_type_l = "car"  # default bucket so it passes TYPES_OK
        # Final gate on types-of-interest (keeps pedestrians/cyclists/cars/trucks/buses)
        if obj_type_l not in TYPES_OK:
            continue


        
        pose = client.simGetObjectPose(idname)
        if pose is None:
            continue

        # Use the already-resolved type string (may be empty)
        prof = H.get_profile_for_idname(idname, obj_type if obj_type else None)
        
        L, W, Hdim = prof["L"], prof["W"], prof["H"]
        z_off   = prof.get("Z", 0.0)
        fwd_off = prof.get("FWD_OFF_M", 0.0)
        adj_pose = H.pose_with_offsets(pose, z_off_m=z_off, fwd_off_m=fwd_off)

        corners_w = H.compute_bounding_box_corners_world(adj_pose, L, W, Hdim)

        if strict_fov:
            # world -> sensor for corners/center
            corners_s = (R_sw @ (corners_w - t_ws).T).T
            center_w = np.array([adj_pose.position.x_val,
                                 adj_pose.position.y_val,
                                 adj_pose.position.z_val], dtype=float)
            center_s = R_sw @ (center_w - t_ws)
            h0, h1 = lidar_cfg["h_start"], lidar_cfg["h_end"]
            v0, v1 = lidar_cfg["v_lower"], lidar_cfg["v_upper"]
            rng    = lidar_cfg["range_m"]
            corner_ok = any(point_in_lidar_fov_sensor_frame(c, h0, h1, v0, v1, rng) for c in corners_s)
            center_ok = point_in_lidar_fov_sensor_frame(center_s, h0, h1, v0, v1, rng)
            if not (corner_ok or center_ok):
                continue

        color_bgr = H.BOX_COLORS_BGR[idx % len(H.BOX_COLORS_BGR)]
        idx += 1
        results.append({
            "found": True,
            "label": label if label else idname,
            "actor_name": idname,
            "actor_pose": pose,
            "adjusted_pose": adj_pose,
            "L": L, "W": W, "H": Hdim,
            "corners_w": corners_w,
            "box_color": color_bgr,
        })
    print(f"[PP] LiDAR FOV selected {len(results)} object(s) for {vehicle_name}.")
    return results


def kitti_json_pp_from_res(res, R_sw, t_ws):
    """
    Build a DAIR-V2X PointPillars-style label in **LiDAR frame**.
    - Location: LiDAR coords (x forward, y left, z up).
    - Rotation: yaw around +Z of LiDAR (i.e., BEV heading).
    """
    if not res or not res.get("found", False):
        return None
    # Dimensions
    h = float(res["H"]); w = float(res["W"]); l = float(res["L"])

    # World -> LiDAR sensor (AirSim frame first)
    center_w = np.array([res["adjusted_pose"].position.x_val,
                         res["adjusted_pose"].position.y_val,
                         res["adjusted_pose"].position.z_val], dtype=float)
    center_s_air = R_sw @ (center_w - t_ws)
    # AirSim (x,y,z) with y right, z down -> DAIR LiDAR (y left, z up)
    center_s = center_s_air.copy()
    center_s[1] = -center_s[1]
    center_s[2] = -center_s[2]

    # --- Yaw in LiDAR: compute in AirSim sensor frame, then flip to Virtual LiDAR ---
    # p_s is center in AirSim LiDAR (x fwd, y right, z down)
    p_s = R_sw @ (center_w - t_ws)
    yaw_lid = math.atan2(p_s[1], p_s[0])  # AirSim sensor-frame yaw
    # Flip to DAIR Virtual LiDAR: y←−y, z←−z ⇒ yaw changes sign
    xv, yv, zv = p_s[0], -p_s[1], -p_s[2]
    rot_z = -yaw_lid

    # Type mapping (consistent with your KITTI writer)
    lbl = str(res.get("label","")).lower()
    tname = "Pedestrian" if (("human" in lbl) or ("pedestrian" in lbl) or H.infer_object_type_from_label(lbl)=="human") else "Car"

    return {
        "type": tname,
        "occluded_state": 0,
        "truncated_state": 0,
        "alpha": -1.0,  # not used for PP; keep a placeholder
        "2d_box": {"xmin": -1, "ymin": -1, "xmax": -1, "ymax": -1},
        "3d_dimensions": {"h": h, "w": w, "l": l},
        "3d_location": {"x": float(center_s[0]), "y": float(center_s[1]), "z": float(center_s[2])},
        "rotation": rot_z
    }


def kitti_json_3donly_from_res(res, cam_pose, P, img_size):
    # 3D-only KITTI/DAIR entry, 2D bbox set to [-1,-1,-1,-1].
    if not res or not res.get("found", False):
        return None
    # 3D dims
    h = float(res["H"]); w = float(res["W"]); l = float(res["L"])
    # camera-frame location
    R_cam = H.quaternion_to_rotation_matrix(cam_pose.orientation)
    cam_p = np.array([cam_pose.position.x_val, cam_pose.position.y_val, cam_pose.position.z_val], dtype=float)
    center_w = np.array([res["adjusted_pose"].position.x_val,
                         res["adjusted_pose"].position.y_val,
                         res["adjusted_pose"].position.z_val], dtype=float)
    center_c = R_cam.T @ (center_w - cam_p)
    # yaw in camera frame from box orientation
    R_box = H.quaternion_to_rotation_matrix(res["adjusted_pose"].orientation)
    fwd_local = np.array([1,0,0], dtype=float)
    fwd_world = R_box @ fwd_local
    fwd_cam   = R_cam.T @ fwd_world
    # Consistent with KITTI rotation_y ↔ AirSim Z-yaw: use atan2(Y, X)
    rot_y = float(math.atan2(fwd_cam[1], fwd_cam[0]))    # type mapping (reuse your heuristic)
    lbl = str(res.get("label","")).lower()
    label_type = "Pedestrian" if (("human" in lbl) or ("pedestrian" in lbl) or H.infer_object_type_from_label(lbl)=="human") else "Car"
    return {
        "type": label_type,
        "occluded_state": 0,
        "truncated_state": 0,
        "alpha": float(rot_y),
        "2d_box": {"xmin": -1, "ymin": -1, "xmax": -1, "ymax": -1},
        "3d_dimensions": {"h": h, "w": w, "l": l},
        "3d_location": {"x": float(center_c[0]), "y": float(center_c[1]), "z": float(center_c[2])},
        "rotation": rot_y
    }


# ------------------ MAIN ------------------
def main():
    # Clients
    drone_client = airsim.MultirotorClient(port=DRONE_PORT)
    car_client   = airsim.CarClient(port=HUSKY_PORT)
    print("[STARTUP] connecting…")
    drone_client.confirmConnection()
    car_client.confirmConnection()
    for n in SIDE_INF: drone_client.enableApiControl(True, vehicle_name=n)
    for n in SIDE_VEH: car_client.enableApiControl(True, vehicle_name=n)
    print("[STARTUP] connected.")

    # Pause and set small detection radii if you use simGetDetections elsewhere (optional)
    drone_client.simPause(True)

    # Calibs (use class method for K)
    veh_calib = build_calib_json(car_client,   SIDE_VEH[0])
    inf_calib = build_calib_json(drone_client, SIDE_INF[0])
    write_calib_json(os.path.join(PATHS["veh"]["calib"], "calib.json"), veh_calib)
    write_calib_json(os.path.join(PATHS["inf"]["calib"], "calib.json"), inf_calib)

    # Actor map
    id_to_label = H.load_actor_map(H.CSV_PATH)   # from your class config

    veh_ts_list, inf_ts_list = [], []

    num_ticks = int(round(DURATION_S / DT))
    for tick in range(1, num_ticks+1):
        # Advance one fixed slice WHILE PAUSED → all sensors & poses are from the same time
        try:
            drone_client.simContinueForTime(DT)  # one step; sim remains paused after
        except Exception as e:
            print(f"[ERROR] sim step failed at tick {tick}: {e}")
            continue

        t_ms = int(round(tick * DT * 1000.0))
        if tick % 100 == 0:
            print(f"[COLLECT] tick {tick}/{num_ticks}")

        # === VEHICLE SIDE ===
        veh_img = veh_depth = veh_seg = None
        veh_pts = None

        if tick % CAM_EVERY == 0:
            veh_img, veh_depth, veh_seg = get_images(car_client, SIDE_VEH[0])
            cv2.imwrite(os.path.join(PATHS["veh"]["img"],   f"{t_ms}{IMG_EXT}"), veh_img)
            np.save    (os.path.join(PATHS["veh"]["depth"], f"{t_ms}.npy"),      veh_depth)
            cv2.imwrite(os.path.join(PATHS["veh"]["seg"],   f"{t_ms}{IMG_EXT}"), veh_seg)

        if tick % LIDAR_EVERY == 0:
            veh_pts = get_lidar_points(car_client, SIDE_VEH[0])
            if veh_pts is not None:
                save_lidar_bin(os.path.join(PATHS["veh"]["lidar"], f"{t_ms}.bin"), veh_pts)
                veh_ts_list.append(t_ms)

        # === INFRA SIDE ===
        inf_img = inf_depth = inf_seg = None
        inf_pts = None

        if tick % CAM_EVERY == 0:
            inf_img, inf_depth, inf_seg = get_images(drone_client, SIDE_INF[0])
            cv2.imwrite(os.path.join(PATHS["inf"]["img"],   f"{t_ms}{IMG_EXT}"), inf_img)
            np.save    (os.path.join(PATHS["inf"]["depth"], f"{t_ms}.npy"),      inf_depth)
            cv2.imwrite(os.path.join(PATHS["inf"]["seg"],   f"{t_ms}{IMG_EXT}"), inf_seg)

        if tick % LIDAR_EVERY == 0:
            inf_pts = get_lidar_points(drone_client, SIDE_INF[0])
            if inf_pts is not None:
                save_lidar_bin(os.path.join(PATHS["inf"]["lidar"], f"{t_ms}.bin"), inf_pts)
                inf_ts_list.append(t_ms)

        # === Label both sides on the SAME paused step ===
        if (tick % CAM_EVERY == 0):
            # Camera infos/poses at this same paused step
            veh_info = car_client.simGetCameraInfo(CAM_NAME,   vehicle_name=SIDE_VEH[0])
            inf_info = drone_client.simGetCameraInfo(CAM_NAME, vehicle_name=SIDE_INF[0])
            cam_pose_veh = veh_info.pose
            cam_pose_inf = inf_info.pose

            # Always use the *live* AirSim projection matrix and actual image size
            P_veh = np.array(veh_info.proj_mat.matrix, dtype=np.float64).reshape((4,4))
            if not (np.isfinite(P_veh).all() and not np.allclose(P_veh, 0)):
                P_veh = np.array(veh_calib["P"], dtype=np.float64)
            W_veh, H_veh = (veh_img.shape[1], veh_img.shape[0]) if veh_img is not None else tuple(veh_calib["image_size"])

            P_inf = np.array(inf_info.proj_mat.matrix, dtype=np.float64).reshape((4,4))
            if not (np.isfinite(P_inf).all() and not np.allclose(P_inf, 0)):
                P_inf = np.array(inf_calib["P"], dtype=np.float64)
            W_inf, H_inf = (inf_img.shape[1], inf_img.shape[0]) if inf_img is not None else tuple(inf_calib["image_size"])

            # Build targets via your class (CSV + scene + FOV/range gating)
            veh_targets = H.build_targets_from_csv_scene(car_client,   id_to_label, cam_pose_veh, P_veh, W_veh, H_veh)
            # For infra: use the same *live* matrices; this fixes pedestrians being pre-gated by a stale P
            inf_targets = H.build_targets_from_csv_scene(drone_client, id_to_label, cam_pose_inf, P_inf, W_inf, H_inf)

            # Process each target using your per-target pipeline
            veh_labels = []
            if veh_img is not None:
                for tgt in veh_targets:
                    res = H.process_target(tgt, car_client, veh_info, cam_pose_veh, veh_img, veh_seg, veh_depth, P_veh)
                    kj = kitti_json_from_result(res, cam_pose_veh, P_veh, (W_veh, H_veh))
                    if kj is not None:
                        veh_labels.append(kj)

            inf_labels = []
            if inf_img is not None:
                for tgt in inf_targets:
                    res = H.process_target(tgt, drone_client, inf_info, cam_pose_inf, inf_img, inf_seg, inf_depth, P_inf)
                    kj = kitti_json_from_result(res, cam_pose_inf, P_inf, (W_inf, H_inf))
                    if kj is not None:
                        inf_labels.append(kj)

            # Write per-timestamp label JSONs (mirrored per side, and “label/{veh,inf}” convenience)
            with open(os.path.join(PATHS["veh"]["kitti_label"], f"{t_ms:06d}.json"), "w") as f:
                json.dump(veh_labels, f)
            with open(os.path.join(PATHS["inf"]["kitti_label"], f"{t_ms:06d}.json"), "w") as f:
                json.dump(inf_labels, f)


                        # --- Build and write PointPillars (LiDAR-gated) labels ---
            # Vehicle side
            try:
                ld_v  = car_client.getLidarData(lidar_name=LIDAR_NAME, vehicle_name=SIDE_VEH[0])
                pts_v = np.array(ld_v.point_cloud, dtype=np.float64).reshape(-1,3) if (ld_v and ld_v.point_cloud) else None
                # Stable world transform from settings extrinsics × live vehicle pose
                R_wl_v, t_wl_v = lidar_world_from_settings(car_client, SIDE_VEH[0], LIDAR_NAME, settings_path=ABS_SETTINGS_PATH)
                
                world_pts_v = ((R_wl_v @ pts_v.T).T + t_wl_v) if (pts_v is not None and R_wl_v is not None) else None
                # sensor←world for corner/FOV tests
                R_sw_v = R_wl_v.T if R_wl_v is not None else None
                t_ws_v = t_wl_v if t_wl_v is not None else None
                
                cfg_v = lidar_fov_from_settings(SIDE_VEH[0], LIDAR_NAME)
                id_to_label = H.load_actor_map(H.CSV_PATH)

                # UGV: skip FOV pre-gate; trust points-in-box (handles tall/thin pedestrians)
                pp_cands_v = build_lidar_candidates(car_client, id_to_label, SIDE_VEH[0],
                                                    R_sw_v, t_ws_v, cfg_v, strict_fov=False) if (R_sw_v is not None) else []                
                
                pp_kept_v = []
                if world_pts_v is not None:
                    for res in pp_cands_v:
                        inside = H.points_inside_oriented_box(world_pts_v, res["adjusted_pose"], res["L"], res["W"], res["H"])
                        if int(inside.sum()) >= 1:
                            res["num_lidar_points"] = int(inside.sum())
                            pp_kept_v.append(res)
                pp_dir_v = PATHS["veh"]["kitti_label_pp"]
                os.makedirs(pp_dir_v, exist_ok=True)
                # Write LiDAR-frame labels for PointPillars (kitti_label_pp)
                out_v = []
                for res in pp_kept_v:
                    jj = kitti_json_pp_from_res(res, R_sw_v, t_ws_v)
                    if jj is not None:
                        out_v.append(jj)
                with open(os.path.join(pp_dir_v, f"{t_ms:06d}.json"), "w") as f:
                    json.dump(out_v, f)
            except Exception as e:
                print("[WARN] vehicle PP label build failed:", e)

            # Infra side
            try:
                ld_i  = drone_client.getLidarData(lidar_name=LIDAR_NAME, vehicle_name=SIDE_INF[0])
                pts_i = np.array(ld_i.point_cloud, dtype=np.float64).reshape(-1,3) if (ld_i and ld_i.point_cloud) else None
                # Keep infra path stable as well (settings extrinsics × live vehicle pose)
                R_wl_i, t_wl_i = lidar_world_from_settings(drone_client, SIDE_INF[0], LIDAR_NAME, settings_path=ABS_SETTINGS_PATH)

                world_pts_i = ((R_wl_i @ pts_i.T).T + t_wl_i) if (pts_i is not None and R_wl_i is not None) else None
                # sensor←world for corner/FOV tests
                R_sw_i = R_wl_i.T if R_wl_i is not None else None
                t_ws_i = t_wl_i if t_wl_i is not None else None

                cfg_i = lidar_fov_from_settings(SIDE_INF[0], LIDAR_NAME)
                id_to_label = H.load_actor_map(H.CSV_PATH)
                
                # Infra already behaves well; keep the angular pre-gate
                pp_cands_i = build_lidar_candidates(drone_client, id_to_label, SIDE_INF[0],
                                                    R_sw_i, t_ws_i, cfg_i, strict_fov=True) if (R_sw_i is not None) else []

                pp_kept_i = []
                if world_pts_i is not None:
                    for res in pp_cands_i:
                        inside = H.points_inside_oriented_box(world_pts_i, res["adjusted_pose"], res["L"], res["W"], res["H"])
                        if int(inside.sum()) >= 1:
                            res["num_lidar_points"] = int(inside.sum())
                            pp_kept_i.append(res)
                pp_dir_i = PATHS["inf"]["kitti_label_pp"]
                os.makedirs(pp_dir_i, exist_ok=True)
                # Write LiDAR-frame labels for PointPillars (kitti_label_pp)
                out_i = []
                for res in pp_kept_i:
                    jj = kitti_json_pp_from_res(res, R_sw_i, t_ws_i)
                    if jj is not None:
                        out_i.append(jj)
                with open(os.path.join(pp_dir_i, f"{t_ms:06d}.json"), "w") as f:
                    json.dump(out_i, f)
            except Exception as e:
                print("[WARN] infra PP label build failed:", e)

            with open(os.path.join(PATHS["label"]["veh"], f"{t_ms:06d}.json"), "w") as f:
                json.dump(veh_labels, f)
            with open(os.path.join(PATHS["label"]["inf"], f"{t_ms:06d}.json"), "w") as f:
                json.dump(inf_labels, f)

    # timestamps
    with open(os.path.join(PATHS["veh"]["ts"], "timestamp.txt"), "w") as f:
        for t in veh_ts_list: f.write(f"{t}\n")
    with open(os.path.join(PATHS["inf"]["ts"], "timestamp.txt"), "w") as f:
        for t in inf_ts_list: f.write(f"{t}\n")

    # Unpause
    drone_client.simPause(False)
    print("[DONE] Collection complete.")

if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
