#!/usr/bin/env python3

import os, json, time, math
import numpy as np
import setup_path
import hercules as airsim
import cv2
from Hercules2D3DDetector import Hercules2D3DDetector as H

# =========================
# Config
# =========================
SELECTED_VEHICLE = "Husky1"      # "Husky1" (UGV) or "Drone1" (UAV)
CAMERA_NAME_OVERRIDE = None      # e.g., "front_center" or None

# Sim step control
ADVANCE_DT_SECONDS = 0.10  # 0.1s per timestep
N_STEPS = 5000

# DAIR-V2X-C-style root
# DAIRV2X_C_ROOT = "/home/sgarimella34/multi-robot-coordination/collaborative-perception-BEVP/datasets/dair_v2x_synth/cooperative-vehicle-infrastructure"
DAIRV2X_C_ROOT = "/media/sgarimella34/hercules-collect/collaborative-perception-BEVP/datasets/dair_v2x_synth_test8/cooperative-vehicle-infrastructure"

# AirSim settings
SETTINGS_JSON_PATH = "/home/sgarimella34/Documents/AirSim/settings.json"

# Names used on both sides
CAM_NAME   = "front_center"
LIDAR_NAME = "LidarSensor1"

# Vehicle names per side in your sim
VEHICLE_SIDE_NAME = "Husky1"
INFRA_SIDE_NAME   = "Drone1"

# =========================
# Helpers: vehicle config
# =========================
def _configure_from_vehicle(vehicle_name: str):
    name_l = (vehicle_name or "").lower()
    if name_l.startswith("husky"):
        platform = "ugv"
        H.CLIENT_CLASS = airsim.CarClient
        H.PORT = 41452
    else:
        platform = "drone"
        H.CLIENT_CLASS = airsim.MultirotorClient
        H.PORT = 41451

    H.VEHICLE_NAME = vehicle_name
    if CAMERA_NAME_OVERRIDE:
        H.CAMERA_NAME = CAMERA_NAME_OVERRIDE

    print(
        f"[CFG] platform={platform}, vehicle={H.VEHICLE_NAME}, "
        f"client={H.CLIENT_CLASS.__name__}, port={H.PORT}, "
        f"camera={getattr(H, 'CAMERA_NAME', 'front_center')}"
    )

def _advance_sim_once(ctrl, dt_sec: float):
    ctrl.simPause(True)
    if hasattr(ctrl, "simContinueForTime"):
        ctrl.simContinueForTime(float(dt_sec))
        ctrl.simPause(True); return
    # Fallback if needed:
    ctrl.simPause(False)
    time.sleep(max(0.0, float(dt_sec)))
    ctrl.simPause(True)

def _client_for_vehicle(vehicle_name: str):
    name_l = (vehicle_name or "").lower()
    if name_l.startswith("husky"):
        cli = airsim.CarClient(port=41452)
    else:
        cli = airsim.MultirotorClient(port=41451)
    cli.confirmConnection()
    return cli

# =========================
# Linear algebra helpers
# =========================
def _deg2rad(d): return d * math.pi / 180.0

def _R_from_rpy_deg(roll_deg, pitch_deg, yaw_deg):
    r, p, y = map(_deg2rad, (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0],[0, cr, -sr],[0, sr,  cr]], dtype=float)
    Ry = np.array([[ cp, 0, sp],[  0, 1,  0],[-sp, 0, cp]], dtype=float)
    Rz = np.array([[cy, -sy, 0],[sy,  cy, 0],[ 0,   0, 1]], dtype=float)
    return Rz @ Ry @ Rx  # AirSim NED: roll X, pitch Y, yaw Z

def _T_from_xyzrpy(x, y, z, roll_deg, pitch_deg, yaw_deg):
    T = np.eye(4, dtype=float)
    T[:3, :3] = _R_from_rpy_deg(roll_deg, pitch_deg, yaw_deg)
    T[:3,  3] = np.array([x, y, z], dtype=float)
    return T

def _quat_to_R(w, x, y, z):
    # AirSim uses (w,x,y,z)
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]
    ], dtype=float)
    return R

def _hat4(R, t):
    T = np.eye(4, dtype=float)
    T[:3,:3] = R
    T[:3, 3] = t
    return T

# AirSim↔DAIR basis change (x same, flip y & z)
_B = np.eye(4, dtype=float)
_B[1,1] = -1.0
_B[2,2] = -1.0

# =========================
# Settings readers
# =========================
def _load_settings(path):
    with open(path, "r") as f:
        return json.load(f)

def _vehicle_spawn_xyz_from_settings(settings_path, vehicle_name):
    try:
        js = _load_settings(settings_path)
        v = js["Vehicles"][vehicle_name]
        return np.array([v.get("X", 0.0), v.get("Y", 0.0), v.get("Z", 0.0)], dtype=float)
    except Exception:
        return np.zeros(3, dtype=float)

def _cam_size_hfov_from_settings(settings_path, vehicle_name, cam_name):
    """
    Return (width, height, hfov_deg) for the Scene image type from settings.json.
    If you prefer the live value from AirSim, you can query:
        info = ctrl.simGetCameraInfo(cam_name, vehicle_name)
        hfov_deg = float(getattr(info, 'fov', hfov_deg))
    """
    js = _load_settings(settings_path)
    v = js["Vehicles"][vehicle_name]
    c = v["Cameras"][cam_name]
    width = height = None
    hfov_deg = None
    for cap in c.get("CaptureSettings", []):
        if int(cap.get("ImageType", 0)) == int(airsim.ImageType.Scene):
            width = int(cap.get("Width", 1920))
            height = int(cap.get("Height", 1080))
            hfov_deg = float(cap.get("FOV_Degrees", 90.0))
            break
    if width is None or height is None:
        width, height = 1920, 1080
    if hfov_deg is None:
        hfov_deg = 90.0
    return width, height, hfov_deg

# =========================
# Extrinsics from settings.json (LiDAR mounts)
# =========================
def T_cam_lidar_from_settings(vehicle_name, cam_name, lidar_name, settings_path):
    """
    Build lidar→camera extrinsics purely from settings.json (vehicle->sensor mounts).
    AirSim stores sensor poses as vehicle→sensor. So:
      T_cam_lidar = inv(T_vehicle→camera) @ T_vehicle→lidar
    (all in AirSim basis; caller can basis-flip to DAIR)
    """
    js = _load_settings(settings_path)
    v = js["Vehicles"][vehicle_name]

    # Camera mount (vehicle→camera)
    c = v["Cameras"][cam_name]
    T_v_c = _T_from_xyzrpy(
        c.get("X", 0.0), c.get("Y", 0.0), c.get("Z", 0.0),
        c.get("Roll", 0.0), c.get("Pitch", 0.0), c.get("Yaw", 0.0)
    )

    # LiDAR mount (vehicle→lidar)
    l = v["Sensors"][lidar_name]
    T_v_l = _T_from_xyzrpy(
        l.get("X", 0.0), l.get("Y", 0.0), l.get("Z", 0.0),
        l.get("Roll", 0.0), l.get("Pitch", 0.0), l.get("Yaw", 0.0)
    )

    # lidar→camera (AirSim basis)
    T_c_l = np.linalg.inv(T_v_c) @ T_v_l
    return T_c_l

def _save_dair_lidar2cam_json(path, T_cam_lidar):
    """
    Save DAIR-style lidar->camera calibration:
      {
        "rotation":    3x3,
        "translation": [tx, ty, tz]
      }
    """
    os.makedirs(os.path.dirname(path), exist_ok=True)
    R = T_cam_lidar[:3, :3]
    t = T_cam_lidar[:3, 3]
    obj = {"rotation": R.tolist(),
           "translation": [float(t[0]), float(t[1]), float(t[2])]}
    with open(path, "w") as f:
        json.dump(obj, f, indent=2)
    print(f"[calib] Wrote {path}")

def _compute_static_lidar2cam_mats():
    """
    Compute once from settings.json:
      - T_cam_lidar for VEHICLE side (in DAIR Virtual LiDAR basis)
      - T_cam_lidar for INFRA side  (in DAIR Virtual LiDAR basis)
    """
    T_c_l_veh = T_cam_lidar_from_settings(VEHICLE_SIDE_NAME, CAM_NAME, LIDAR_NAME, SETTINGS_JSON_PATH)
    T_c_l_inf = T_cam_lidar_from_settings(INFRA_SIDE_NAME,   CAM_NAME, LIDAR_NAME, SETTINGS_JSON_PATH)

    # Basis change on the LiDAR side only: AirSim (x fwd, y right, z down)
    # -> DAIR Virtual LiDAR (x fwd, y left, z up)
    # T_cam←vl = T_cam←l_AS * B
    T_c_vl_veh = T_c_l_veh @ _B
    T_c_vl_inf = T_c_l_inf @ _B
    return T_c_vl_veh, T_c_vl_inf

def _write_per_frame_lidar_to_camera(frame_id: str, T_c_vl_veh: np.ndarray, T_c_vl_inf: np.ndarray):
    """
    Write DAIR-style lidar->camera JSONs for BOTH sides using the provided frame_id.
    """
    veh_dir = os.path.join(DAIRV2X_C_ROOT, "vehicle-side", "calib", "lidar_to_camera")
    inf_dir = os.path.join(DAIRV2X_C_ROOT, "infrastructure-side", "calib", "virtuallidar_to_camera")
    _save_dair_lidar2cam_json(os.path.join(veh_dir, f"{frame_id}.json"), T_c_vl_veh)
    _save_dair_lidar2cam_json(os.path.join(inf_dir, f"{frame_id}.json"), T_c_vl_inf)

# =========================
# NEW: world extrinsics (DAIR basis)
# =========================
def _airsim_vehicle_pose_T_world_vehicle(ctrl, vehicle_name):
    """
    Return T_world<-vehicle in AirSim basis (x fwd, y right, z down).
    We use kinematics_estimated and then add spawn translation offset
    so translation is absolute (not just relative to start).
    """
    R_wv = np.eye(3, dtype=float)
    t_wv = np.zeros(3, dtype=float)
    try:
        if hasattr(ctrl, "getCarState"):
            st = ctrl.getCarState(vehicle_name=vehicle_name)
            pos = st.kinematics_estimated.position
            ori = st.kinematics_estimated.orientation
        else:
            st = ctrl.getMultirotorState(vehicle_name=vehicle_name)
            pos = st.kinematics_estimated.position
            ori = st.kinematics_estimated.orientation
        R_wv = _quat_to_R(ori.w_val, ori.x_val, ori.y_val, ori.z_val)
        t_wv = np.array([pos.x_val, pos.y_val, pos.z_val], dtype=float)
    except Exception:
        # last resort: simGetVehiclePose
        try:
            vp = ctrl.simGetVehiclePose(vehicle_name=vehicle_name)
            R_wv = _quat_to_R(vp.orientation.w_val, vp.orientation.x_val, vp.orientation.y_val, vp.orientation.z_val)
            t_wv = np.array([vp.position.x_val, vp.position.y_val, vp.position.z_val], dtype=float)
        except Exception:
            pass

    # Add spawn translation from settings.json (if any)
    t_spawn = _vehicle_spawn_xyz_from_settings(SETTINGS_JSON_PATH, vehicle_name)
    t_wv = t_wv + t_spawn
    return _hat4(R_wv, t_wv)

def _T_vehicle_lidar_from_settings(vehicle_name, lidar_name, settings_path):
    js = _load_settings(settings_path)
    v = js["Vehicles"][vehicle_name]
    l = v["Sensors"][lidar_name]
    return _T_from_xyzrpy(
        l.get("X", 0.0), l.get("Y", 0.0), l.get("Z", 0.0),
        l.get("Roll", 0.0), l.get("Pitch", 0.0), l.get("Yaw", 0.0)
    )  # AirSim basis

def _save_RT_json(path, T_dest_src_DAIR):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    R = T_dest_src_DAIR[:3,:3]
    t = T_dest_src_DAIR[:3, 3]
    obj = {"rotation": R.tolist(),
           "translation": [float(t[0]), float(t[1]), float(t[2])]}
    with open(path, "w") as f:
        json.dump(obj, f, indent=2)
    print(f"[calib] Wrote {path}")

def _write_world_extrinsics_for_frame(frame_id, ctrl_veh, ctrl_inf):
    """
    Writes:
      - infrastructure-side/calib/virtuallidar_to_world/{frame}.json   (T_world<-vl)
      - vehicle-side/calib/lidar_to_novatel/{frame}.json               (T_novatel<-lidar), NovAtel≡vehicle
      - vehicle-side/calib/novatel_to_world/{frame}.json               (T_world<-novatel)
    All in DAIR basis (x fwd, y left, z up).
    """
    # === Infrastructure (assume "infrastructure vehicle" = INFRA_SIDE_NAME) ===
    T_w_v_AS_inf = _airsim_vehicle_pose_T_world_vehicle(ctrl_inf, INFRA_SIDE_NAME)  # AirSim basis
    T_v_l_AS_inf = _T_vehicle_lidar_from_settings(INFRA_SIDE_NAME, LIDAR_NAME, SETTINGS_JSON_PATH)
    T_w_l_AS_inf = T_w_v_AS_inf @ T_v_l_AS_inf

    # Basis change: DAIR = B * AirSim * B  (since B == B^{-1})
    T_w_l_DAIR_inf = _B @ T_w_l_AS_inf @ _B

    inf_dir_w = os.path.join(DAIRV2X_C_ROOT, "infrastructure-side", "calib", "virtuallidar_to_world")
    _save_RT_json(os.path.join(inf_dir_w, f"{frame_id}.json"), T_w_l_DAIR_inf)

    # === Vehicle side (NovAtel ≡ vehicle body) ===
    T_w_v_AS_veh = _airsim_vehicle_pose_T_world_vehicle(ctrl_veh, VEHICLE_SIDE_NAME)  # AirSim basis
    T_v_l_AS_veh = _T_vehicle_lidar_from_settings(VEHICLE_SIDE_NAME, LIDAR_NAME, SETTINGS_JSON_PATH)

    # lidar -> novatel (vehicle): T_novatel<-lidar = T_vehicle<-lidar (basis-converted)
    T_n_l_AS_veh = T_v_l_AS_veh
    T_n_l_DAIR_veh = _B @ T_n_l_AS_veh @ _B

    # novatel -> world: T_world<-novatel = T_world<-vehicle (basis-converted)
    T_w_n_AS_veh = T_w_v_AS_veh
    T_w_n_DAIR_veh = _B @ T_w_n_AS_veh @ _B

    veh_dir_nl = os.path.join(DAIRV2X_C_ROOT, "vehicle-side", "calib", "lidar_to_novatel")
    veh_dir_nw = os.path.join(DAIRV2X_C_ROOT, "vehicle-side", "calib", "novatel_to_world")
    _save_RT_json(os.path.join(veh_dir_nl, f"{frame_id}.json"), T_n_l_DAIR_veh)
    _save_RT_json(os.path.join(veh_dir_nw, f"{frame_id}.json"), T_w_n_DAIR_veh)

# =========================
# NEW: intrinsics (DAIR/ROS CameraInfo-style)
# =========================
def _K_from_hfov(width, height, hfov_deg):
    """Rectilinear pinhole, square pixels: fx=fy=(W/2)/tan(hfov/2)."""
    fx = (width / 2.0) / math.tan(_deg2rad(hfov_deg) / 2.0)
    fy = fx  # with square pixels and rectilinear lens
    cx = width  / 2.0
    cy = height / 2.0
    K = np.array([[fx, 0.0, cx],
                  [0.0, fy, cy],
                  [0.0, 0.0, 1.0]], dtype=float)
    return K

def _camera_info_json(width, height, K):
    """
    Build a DAIR/ROS-like CameraInfo JSON.
    Includes both cam_K/K, cam_D/D, R, P, width, height, etc.
    """
    # Distortion disabled in your pipeline
    D = [0.0, 0.0, 0.0, 0.0, 0.0]
    R = np.eye(3, dtype=float)
    # Projection (no baseline)
    P = np.zeros((3,4), dtype=float)
    P[0,0] = K[0,0]; P[0,2] = K[0,2]
    P[1,1] = K[1,1]; P[1,2] = K[1,2]
    P[2,2] = 1.0

    # Flatten helpers
    def fl3x3(m): return [float(x) for x in m.reshape(-1).tolist()]
    def fl3x4(m): return [float(x) for x in m.reshape(-1).tolist()]

    obj = {
        # Minimal DAIR fields many loaders look for:
        "cam_K": fl3x3(K),
        "cam_D": D,
        # Full ROS-style fields (mirroring real DAIR JSONs):
        "width": int(width),
        "height": int(height),
        "distortion_model": "plumb_bob",
        "K": fl3x3(K),
        "D": D,
        "R": fl3x3(R),
        "P": fl3x4(P),
        "binning_x": 0,
        "binning_y": 0,
        "roi": {"x_offset": 0, "y_offset": 0, "height": 0, "width": 0, "do_rectify": False},
        "header": {
            "seq": 0,
            "stamp": {"secs": 0, "nsecs": 0},
            "frame_id": ""
        }
    }
    return obj

def _write_intrinsics_for_frame(frame_id: str, vehicle_name: str, cam_name: str, side_dir: str):
    width, height, hfov_deg = _cam_size_hfov_from_settings(SETTINGS_JSON_PATH, vehicle_name, cam_name)
    K = _K_from_hfov(width, height, hfov_deg)
    obj = _camera_info_json(width, height, K)
    out_dir = os.path.join(DAIRV2X_C_ROOT, side_dir, "calib", "camera_intrinsic")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"{frame_id}.json")
    with open(out_path, "w") as f:
        json.dump(obj, f, indent=2)
    print(f"[intrinsics] Wrote {out_path} (HFOV={hfov_deg:.4f}°, {width}x{height})")

# =========================
# RGB & LiDAR saving (DAIR/KITTI style)
# =========================
def _ensure_dirs():
    paths = [
        os.path.join(DAIRV2X_C_ROOT, "vehicle-side", "image"),
        os.path.join(DAIRV2X_C_ROOT, "vehicle-side", "velodyne"),
        os.path.join(DAIRV2X_C_ROOT, "vehicle-side", "calib", "camera_intrinsic"),
        os.path.join(DAIRV2X_C_ROOT, "infrastructure-side", "image"),
        os.path.join(DAIRV2X_C_ROOT, "infrastructure-side", "velodyne"),
        os.path.join(DAIRV2X_C_ROOT, "infrastructure-side", "calib", "camera_intrinsic"),
        os.path.join(DAIRV2X_C_ROOT, "vehicle-side", "label", "camera"),
        os.path.join(DAIRV2X_C_ROOT, "vehicle-side", "label", "lidar"),
        os.path.join(DAIRV2X_C_ROOT, "infrastructure-side", "label", "camera"),
        os.path.join(DAIRV2X_C_ROOT, "infrastructure-side", "label", "lidar"),
    ]
    for p in paths:
        os.makedirs(p, exist_ok=True)

def _save_png_from_response(resp, path_png):
    arr = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError(f"Failed to decode image for {path_png}")
    cv2.imwrite(path_png, img)

def _capture_rgb_png(ctrl, vehicle_name: str, camera_name: str, path_png: str):
    req = [airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, True)]
    resp = ctrl.simGetImages(req, vehicle_name=vehicle_name)[0]
    _save_png_from_response(resp, path_png)

def _capture_lidar_bin(ctrl, vehicle_name: str, lidar_name: str, path_bin: str):
    """
    Save LiDAR as KITTI-style .bin: Nx4 float32 [x, y, z, intensity].
    AirSim LiDAR points are in sensor-local frame (NED-like x fwd, y right, z down).
    Convert to DAIR/KITTI frame (x fwd, y left, z up) by flipping Y and Z.
    Intensity is set to 1.0 if not provided.
    """
    try:
        ld = ctrl.getLidarData(lidar_name, vehicle_name)
    except TypeError:
        ld = ctrl.getLidarData(lidar_name)
    pts = np.array(ld.point_cloud, dtype=np.float32).reshape(-1, 3) if ld and len(ld.point_cloud) >= 3 else np.zeros((0,3), dtype=np.float32)

    if pts.shape[0] > 0:
        pts[:, 1] *= -1.0  # flip Y
        pts[:, 2] *= -1.0  # flip Z
        intensity = np.ones((pts.shape[0], 1), dtype=np.float32)
        out = np.hstack([pts.astype(np.float32), intensity])
    else:
        out = np.zeros((0,4), dtype=np.float32)

    out.tofile(path_bin)

def _save_rgb_and_lidar_for_frame(frame_id: str, ctrl_veh, ctrl_inf):
    # Ensure dirs exist
    _ensure_dirs()
    veh_img = os.path.join(DAIRV2X_C_ROOT, "vehicle-side", "image",    f"{frame_id}.png")
    veh_bin = os.path.join(DAIRV2X_C_ROOT, "vehicle-side", "velodyne", f"{frame_id}.bin")
    inf_img = os.path.join(DAIRV2X_C_ROOT, "infrastructure-side", "image",    f"{frame_id}.png")
    inf_bin = os.path.join(DAIRV2X_C_ROOT, "infrastructure-side", "velodyne", f"{frame_id}.bin")

    # Capture while paused (same instant)
    _capture_rgb_png(ctrl_veh, VEHICLE_SIDE_NAME, CAM_NAME, veh_img)
    _capture_lidar_bin(ctrl_veh, VEHICLE_SIDE_NAME, LIDAR_NAME, veh_bin)

    _capture_rgb_png(ctrl_inf, INFRA_SIDE_NAME, CAM_NAME, inf_img)
    _capture_lidar_bin(ctrl_inf, INFRA_SIDE_NAME, LIDAR_NAME, inf_bin)

# =========================
# Main
# =========================
def main():
    # Informational header based on SELECTED_VEHICLE
    _configure_from_vehicle(SELECTED_VEHICLE)

    # Create real control clients for BOTH sides (to manage pause/step)
    ctrl_veh = _client_for_vehicle(VEHICLE_SIDE_NAME)
    ctrl_inf = _client_for_vehicle(INFRA_SIDE_NAME)
    print("Connected to simulator (both sides). Starting detection loop...")

    # FrozenClient factories to block any internal unpause during detector.run()
    def make_frozen_client(ctrl):
        class FrozenClient:
            def __init__(self, *args, **kwargs):
                self._c = ctrl
            def simPause(self, is_paused: bool):
                if is_paused:
                    return self._c.simPause(True)
                return None  # swallow unpause attempts
            def __getattr__(self, name):
                return getattr(self._c, name)
        return FrozenClient

    FrozenVeh = make_frozen_client(ctrl_veh)
    FrozenInf = make_frozen_client(ctrl_inf)

    detector = H()

    # Pre-compute static lidar->camera mats (from settings.json, with DAIR basis)
    T_c_vl_veh, T_c_vl_inf = _compute_static_lidar2cam_mats()

    # Start paused on both controllers
    ctrl_veh.simPause(True)
    ctrl_inf.simPause(True)

    for t in range(N_STEPS):
        frame_id = f"{t:06d}"  # KITTI-style zero-padded

        # Write DAIR-style per-frame extrinsics:
        #   lidar->camera (both sides)
        _write_per_frame_lidar_to_camera(frame_id, T_c_vl_veh, T_c_vl_inf)
        #   virtuallidar->world (infra) and vehicle's novatel chain
        _write_world_extrinsics_for_frame(frame_id, ctrl_veh, ctrl_inf)
        #   camera intrinsics (both sides)
        _write_intrinsics_for_frame(frame_id, VEHICLE_SIDE_NAME, CAM_NAME, "vehicle-side")
        _write_intrinsics_for_frame(frame_id, INFRA_SIDE_NAME,  CAM_NAME, "infrastructure-side")

        # Save RGB and LiDAR for BOTH sides at the SAME paused instant
        _save_rgb_and_lidar_for_frame(frame_id, ctrl_veh, ctrl_inf)

        print(f"\n=== Processing timestep {t} (frame_id={frame_id}) ===")
        ctrl_veh.simPause(True); ctrl_inf.simPause(True)

        # VEHICLE SIDE (visualization only; still paused)
        print(f"--- Vehicle-side: {VEHICLE_SIDE_NAME} ---")
        H.VEHICLE_NAME = VEHICLE_SIDE_NAME
        H.PORT = 41452  # informational; FrozenVeh ignores port
        H.CLIENT_CLASS = FrozenVeh
        if CAMERA_NAME_OVERRIDE:
            H.CAMERA_NAME = CAMERA_NAME_OVERRIDE

        veh_cam_dir = os.path.join(DAIRV2X_C_ROOT, "vehicle-side", "label", "camera")
        veh_lid_dir = os.path.join(DAIRV2X_C_ROOT, "vehicle-side", "label", "lidar")

        detector.SAVE_LABELS = True
        detector.FRAME_ID = frame_id
        detector.LABEL_CAMERA_DIR = veh_cam_dir
        detector.LABEL_LIDAR_DIR  = veh_lid_dir
        detector.LIDAR_LABEL_REQUIRE_POINTS = True  # only keep boxes with ≥1 LiDAR return

        detector.run()  # blocks until you close all VEHICLE windows

        # Still paused
        ctrl_veh.simPause(True); ctrl_inf.simPause(True)

        # INFRASTRUCTURE SIDE (visualization only; still paused)
        print(f"--- Infrastructure-side: {INFRA_SIDE_NAME} ---")
        H.VEHICLE_NAME = INFRA_SIDE_NAME
        H.PORT = 41451  # informational; FrozenInf ignores port
        H.CLIENT_CLASS = FrozenInf
        if CAMERA_NAME_OVERRIDE:
            H.CAMERA_NAME = CAMERA_NAME_OVERRIDE

        inf_cam_dir = os.path.join(DAIRV2X_C_ROOT, "infrastructure-side", "label", "camera")
        inf_lid_dir = os.path.join(DAIRV2X_C_ROOT, "infrastructure-side", "label", "lidar")

        detector.SAVE_LABELS = True
        detector.FRAME_ID = frame_id
        detector.LABEL_CAMERA_DIR = inf_cam_dir
        detector.LABEL_LIDAR_DIR  = inf_lid_dir
        detector.LIDAR_LABEL_REQUIRE_POINTS = True

        detector.run()  # blocks until you close all INFRA windows

        # Ensure paused after both sides
        ctrl_veh.simPause(True); ctrl_inf.simPause(True)

        # Advance exactly one step using ONLY the DRONE (infrastructure) client
        if t < N_STEPS - 1:
            _advance_sim_once(ctrl_inf, ADVANCE_DT_SECONDS)
            ctrl_veh.simPause(True)
        else:
            ctrl_veh.simPause(True); ctrl_inf.simPause(True)
            print("Completed all timesteps.")

if __name__ == "__main__":
    main()
