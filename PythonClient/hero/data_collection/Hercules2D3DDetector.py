#!/usr/bin/env python3
"""
Auto-select targets from CSV by actor_label keywords, verify they are in the
current camera FOV and within range, then run the SAME per-target pipeline
you already had (3D cuboid + amodal 2D box + ROI->Open3D + optional tight refit).

Also shows a full-frame segmentation image clipped by depth <= DEPTH_CLIP_MAX_M.

IMPORTANT: All camera data AND all object poses are sampled during a SINGLE
simPause(True) interval. No additional pauses occur after that, so all 3D boxes
correspond to the same simulation time-step.

MOD: Also samples LiDAR in the SAME pause window and visualizes it in Open3D.
"""

import math, re
import numpy as np
import setup_path
import hercules as airsim
import cv2
import csv
from collections import defaultdict
from typing import Tuple
import os, json, math

# optional visualization
try:
    import open3d as o3d
except ImportError:
    o3d = None


class Hercules2D3DDetector:
    """
    Class-based refactor of the original 2D & 3D object detection script.

    This class preserves the original behavior and outputs while enabling clean imports and reuse.
    The configuration values below mirror the originals as class attributes.
    """
    # === CONFIGURATION ===
    TARGETS = []  # will be built automatically from CSV + scene + FOV
    
    CAMERA_NAME        = "front_center"
    CLIENT_CLASS       = airsim.MultirotorClient
    PORT               = 41451
    
    PROJECTION_ENABLED = True
    
    # Draw control:
    DRAW_ONLY_CORRECTED_2D = True
    TIGHT_BOX_COLOR_BGR    = (0,165,255)   # orange for tight (corrected) box
    
    # Dominant color logic & occlusion filter for 2D drawing
    DOMINANT_COLOR_ONLY = True
    
    # Show full-frame segmentation with small-color pruning?
    SEG_FULL_PRUNE_COLORS = False  # set True to prune tiny color islands; False = show all colors
    
    # --- 2D bbox size gating (after color selection) ---
    DOMINANT_MIN_PIXELS = 120
    MIN_BBOX_WIDTH      = 20
    MIN_BBOX_HEIGHT     = 20
    MIN_BBOX_AREA       = 400
    
    # --- per-object profiles (dimensions in meters; Z is NED +Z down)
    PROFILES = {
        "human": {"L": 0.5, "W": 0.75, "H": 1.9,  "Z": -0.90},
        "car":   {"L": 4.2, "W": 1.90, "H": 1.60, "Z": -0.55},
    }
    
    PROFILE_OVERRIDES_BY_ID_SUBSTR = {
        "pickup": {"L": 5.35, "W": 2.05, "H": 2.2, "Z": -1.0, "FWD_OFF_M": -0.45},
    }
    
    SHOW_ROI_WINDOWS_GLOBAL = False  # avoid N windows when many auto targets are found
    
    # --- Depth-clip settings ---
    RANGE_MAX_M   = 40.0
    DEPTH_CLIP_ENABLE     = True
    DEPTH_CLIP_MAX_M      = RANGE_MAX_M   # keep in sync with detection range
    SHOW_ORIGINAL_SEG_ROI = False
    
    # --- Full-frame segmentation color filtering ---
    MIN_SEG_COLOR_PIXELS_FULL = 2000
    
    # --- Point cloud export from clipped ROI ---
    ADD_ROI_POINTS_TO_OPEN3D = True
    ROI_POINT_STRIDE          = 1
    
    # --- tight-box refit params ---
    REFIT_USE_DEPTH_CLIP_FOR_TIGHT_BOX = True
    REFIT_MIN_PIXELS                   = 50
    REFIT_SEARCH_MARGIN_PX             = 20
    
    # --- visible-objects print controls ---
    VISIBLE_EPS_METERS = 1.0
    MAX_VISIBLE_PRINT  = 200

    # === LABEL IO (defaults off) ===
    SAVE_LABELS: bool = False                 # enable/disable writing
    LABEL_CAMERA_DIR: str | None = None       # .../vehicle-side/label/camera   or .../infrastructure-side/label/camera
    LABEL_LIDAR_DIR: str | None = None        # .../vehicle-side/label/lidar    or .../infrastructure-side/label/lidar
    FRAME_ID: str | None = None               # "%06d" from the caller
    LIDAR_LABEL_REQUIRE_POINTS: bool = True   # keep only boxes with >=1 LiDAR point inside in LiDAR label
    MIN_LIDAR_POINTS_IN_BOX = 10 # Minimum LiDAR points required inside a 3D box to keep/save the label
    
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
    
    # ---- Force-included actors ----
    FORCE_INCLUDE_IDNAMES = [
        "BP_VehicleAI_pickup_C_UAID_6C6E07132D49788102_1328099840",
    ]
    FORCE_INCLUDE_OBJECT_TYPE = "car"
    FORCE_INCLUDE_COLOR_BGR   = (0, 0, 255)    # red, reserved for this actor

    # === LiDAR: ===
    # Names align with your settings.json; change VEHICLE_NAME to "Husky1" if needed.
    VEHICLE_NAME = "Drone1"
    LIDAR_NAME   = "LidarSensor1"
    ADD_LIDAR_TO_OPEN3D = True
    LIDAR_STRIDE = 1              # 1 = keep all points; >1 to subsample
    LIDAR_VOXEL_DOWNSAMPLE = None # meters; set to 0 or None to disable
    LIDAR_COLOR_RGB = (1.0, 0.0, 0.0)  # red
    CAMERA_LABEL_REQUIRE_POINTS: bool = False  # set True to match LiDAR counts


    # === LiDAR config from AirSim settings.json (absolute path) ===
    SETTINGS_JSON_PATH = "/home/sgarimella34/Documents/AirSim/settings.json"

    # Global toggle to show any UI (OpenCV windows, Open3D visualizers)
    SHOW_VISUALS = False

    # ===================== helpers =====================
    @staticmethod
    def load_vehicle_spawn_translation(settings_path, vehicle_name):
        """
        Read the vehicle's starting translation (X,Y,Z) from AirSim settings.json.
        Returns a tuple (x0, y0, z0) in meters (NED: +x forward, +y right, +z down).
        Fallback to (0,0,0) if not found or file not available.
        """
        import json
        try:
            with open(settings_path, "r") as f:
                js = json.load(f)
            v = js["Vehicles"][vehicle_name]
            return (
                float(v.get("X", 0.0)),
                float(v.get("Y", 0.0)),
                float(v.get("Z", 0.0)),
            )
        except Exception as e:
            print(f"[WARN] Could not load spawn translation for '{vehicle_name}' "
                  f"from {settings_path}: {e} (using 0,0,0)")
            return (0.0, 0.0, 0.0)

    @staticmethod
    def pose_add_translation(base_pose, delta_xyz):
        """
        Return a new Pose with base_pose's orientation and base_pose.position + delta_xyz.
        """
        dx, dy, dz = (float(delta_xyz[0]), float(delta_xyz[1]), float(delta_xyz[2]))
        p = base_pose.position
        return airsim.Pose(
            position_val=airsim.Vector3r(p.x_val + dx, p.y_val + dy, p.z_val + dz),
            orientation_val=base_pose.orientation
        )


    @staticmethod
    def _angle_in_interval_deg(a, start, end):
        """
        Return True if angle a (deg) is within [start, end] on a circular domain.
        Robust to wrap-around AND full-360° intervals (e.g., 0..360).
        """
        # Span in degrees keeping original difference to detect 360 exactly.
        span_raw = end - start
        span = (span_raw % 360.0 + 360.0) % 360.0  # [0, 360)
        # If the raw interval is exactly (or numerically) 360°, accept all angles.
        if math.isclose(span, 0.0, abs_tol=1e-6) and not math.isclose(span_raw, 0.0, abs_tol=1e-6):
            return True
        # Reduce to a relative test from 'start'
        rel = ((a - start) % 360.0 + 360.0) % 360.0  # [0, 360)
        return rel <= span


    @staticmethod
    def build_lidar_targets(client, id_to_label, R_sw, t_ws, lidar_cfg):
        """
        Selects objects of interest by LiDAR cone only (keywords + LiDAR FOV/Range).
        Returns a list of lightweight 'res-like' dicts (same fields needed for Open3D box draw).
        """
        results_lidar = []
        try:
            scene_ids = list(client.simListSceneObjects(".*"))
        except Exception as e:
            print("simListSceneObjects failed:", e)
            scene_ids = []

        # Pull config pieces:
        h0 = lidar_cfg["h_start_deg"]; h1 = lidar_cfg["h_end_deg"]
        v0 = lidar_cfg["v_lower_deg"]; v1 = lidar_cfg["v_upper_deg"]
        rng = lidar_cfg["range_m"]

        for idx, idname in enumerate(scene_ids):
            label = Hercules2D3DDetector.load_actor_map  # (silence lints; real call below)
            label = id_to_label.get(idname, "")
            if not (Hercules2D3DDetector.label_has_keyword(label, Hercules2D3DDetector.KEYWORDS) or
                    Hercules2D3DDetector.label_has_keyword(idname, Hercules2D3DDetector.KEYWORDS)):
                continue

            pose = Hercules2D3DDetector._safe_get_pose(client, idname)
            if pose is None:
                continue

            # Use same type/profile logic you already use:
            obj_type = Hercules2D3DDetector.infer_object_type_from_label(label if label else idname)
            prof = Hercules2D3DDetector.get_profile_for_idname(idname, obj_type)
            L, W, H = prof["L"], prof["W"], prof["H"]
            z_off   = prof.get("Z", 0.0)
            fwd_off = prof.get("FWD_OFF_M", 0.0)
            adj_pose = Hercules2D3DDetector.pose_with_offsets(pose, z_off_m=z_off, fwd_off_m=fwd_off)

            # Cuboid corners in world, then into LiDAR sensor frame:
            corners_w = Hercules2D3DDetector.compute_bounding_box_corners_world(adj_pose, L, W, H)
            corners_s = (R_sw @ (corners_w - t_ws).T).T  # world -> sensor

            # LiDAR cone test: any corner inside FOV/Range is enough
            inside = False
            for c in corners_s:
                if Hercules2D3DDetector.point_in_lidar_fov_sensor_frame(c, h0, h1, v0, v1, rng):
                    inside = True
                    break
            if not inside:
                continue

            # Assign a stable color from your palette:
            color_bgr = Hercules2D3DDetector.BOX_COLORS_BGR[idx % len(Hercules2D3DDetector.BOX_COLORS_BGR)]
            results_lidar.append({
                "found": True,
                "label": label if label else idname,
                "actor_name": idname,
                "actor_pose": pose,
                "adjusted_pose": adj_pose,
                "L": L, "W": W, "H": H,
                "corners_w": corners_w,
                "box_color": color_bgr,
                "drew_tight": False  # not relevant for LiDAR-only additions
            })
        print(f"LiDAR-selected {len(results_lidar)} target(s) by LiDAR FOV/Range only.")
        return results_lidar


    @staticmethod
    def point_in_lidar_fov_sensor_frame(p_s, h_start_deg, h_end_deg, v_lower_deg, v_upper_deg, range_m):
        """
        p_s: (3,) point in LiDAR sensor frame.
        AirSim sensor/body frames follow NED: x forward, y right, z down.
        We flip signs on y and z when computing angles to align with a +left/+up convention.
        Checks spherical angles and range against the LiDAR viewing cone.
        """

        x, y, z = float(p_s[0]), float(p_s[1]), float(p_s[2])
        r = math.sqrt(x*x + y*y + z*z)
        if not (math.isfinite(r) and r > 0.0 and r <= range_m):
            return False

        # Horizontal (yaw) angle: +left   (AirSim has y to the RIGHT -> flip y)
        yaw_deg = math.degrees(math.atan2(-y, x))
        # Vertical angle: +up from the horizontal plane   (AirSim has z DOWN -> flip z)
        v_deg = math.degrees(math.atan2(-z, math.hypot(x, y)))

        if not Hercules2D3DDetector._angle_in_interval_deg(yaw_deg, h_start_deg, h_end_deg):
            return False
        return (v_deg >= v_lower_deg) and (v_deg <= v_upper_deg)


    @staticmethod
    def load_lidar_fov_from_settings(settings_path, vehicle_name, lidar_name):
        import json, os
        # Defaults in case something is missing:
        cfg = {
            "range_m": float("inf"),
            "h_start_deg": -180.0,
            "h_end_deg":   180.0,
            "v_lower_deg": -90.0,
            "v_upper_deg":  90.0,
        }
        try:
            with open(settings_path, "r") as f:
                js = json.load(f)
            sensor = js["Vehicles"][vehicle_name]["Sensors"][lidar_name]
            cfg["range_m"]   = float(sensor.get("Range", cfg["range_m"]))
            cfg["h_start_deg"] = float(sensor.get("HorizontalFOVStart", cfg["h_start_deg"]))
            cfg["h_end_deg"]   = float(sensor.get("HorizontalFOVEnd",   cfg["h_end_deg"]))
            cfg["v_lower_deg"] = float(sensor.get("VerticalFOVLower",   cfg["v_lower_deg"]))
            cfg["v_upper_deg"] = float(sensor.get("VerticalFOVUpper",   cfg["v_upper_deg"]))
        except Exception as e:
            print(f"[WARN] Could not load LiDAR FOV from {settings_path}: {e} (using defaults)")
        return cfg


    @staticmethod
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
    @staticmethod
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
    @staticmethod
    def infer_object_type_from_label(label):
        s = (label or "").lower()
        if ("human" in s) or ("person" in s) or ("pedestrian" in s) or ("splinehuman" in s):
            return "human"
        if any(k in s for k in ("car","truck","sedan","suv","vehicle","van","bus","sportscar","policecar","pickup")):
            return "car"
        return "car"
    @staticmethod
    def cam_to_point_range(pt_world, cam_pose):
        R_cam = quaternion_to_rotation_matrix(cam_pose.orientation)
        cam_p = np.array([cam_pose.position.x_val, cam_pose.position.y_val, cam_pose.position.z_val], dtype=float)
        p_cam = R_cam.T @ (pt_world - cam_p)
        return float(np.linalg.norm(p_cam)), p_cam[0]
    @staticmethod
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
    @staticmethod
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
    @staticmethod
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
    @staticmethod
    def compute_intrinsics_from_horizontal_fov(hfov_deg, width, height):
        hfov = math.radians(hfov_deg)
        fx = (width/2.0) / math.tan(hfov/2.0)
        fy = fx
        cx, cy = width/2.0, height/2.0
        K = np.array([[fx, 0, cx],[0, fy, cy],[0, 0, 1]], dtype=float)
        vfov = 2 * math.degrees(math.atan((height/2.0)/fy))
        return K, vfov
    @staticmethod
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
    @staticmethod
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
    @staticmethod
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
    @staticmethod
    def resize_to(img, target_w, target_h, is_depth=False):
        if img.shape[1] == target_w and img.shape[0] == target_h:
            return img
        return cv2.resize(img, (target_w, target_h), interpolation=cv2.INTER_NEAREST)
    @staticmethod
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
    @staticmethod
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
        # Note: using DepthPerspective as forward depth; if needed, convert to ray-range.
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
    @staticmethod
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
    @staticmethod
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
    @staticmethod
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
    @staticmethod
    def world_to_cam_and_pixel(pt_world, cam_pose, P, img_w, img_h):
        R_cam = quaternion_to_rotation_matrix(cam_pose.orientation)
        cam_p = np.array([cam_pose.position.x_val, cam_pose.position.y_val, cam_pose.position.z_val], dtype=float)
        p_cam = R_cam.T @ (pt_world - cam_p)
        in_front = p_cam[0] > 1e-6
        pts_h = np.array([[p_cam[0], p_cam[1], p_cam[2], 1.0]], dtype=float)
        clip  = (P @ pts_h.T).T
        ndc   = (clip[:, :3] / clip[:, 3:4])[0]
        u = (1.0 - (ndc[0] * 0.5 + 0.5)) * img_w
        v = (ndc[1] * 0.5 * 1.0 + 0.5) * img_h
        in_bounds = (u >= 0) and (u < img_w) and (v >= 0) and (v < img_h)
        return p_cam, float(u), float(v), bool(in_front), bool(in_bounds)
    @staticmethod
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
    @staticmethod
    def _safe_get_pose(client, name):
        """Try newer signature with 'True', fall back if unsupported."""
        try:
            return client.simGetObjectPose(name, True)
        except TypeError:
            return client.simGetObjectPose(name)
    @staticmethod
    def resolve_id_exact_or_prefix(client, idname):
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
    @staticmethod
    def get_profile_for_idname(idname, obj_type):
        prof = PROFILES.get(obj_type, PROFILES["car"]).copy()
        lname = idname.lower()
        for substr, override in PROFILE_OVERRIDES_BY_ID_SUBSTR.items():
            if substr in lname:
                prof.update(override)
                break
        if "FWD_OFF_M" not in prof:
            prof["FWD_OFF_M"] = 0.0
        return prof
    @staticmethod
    def pose_with_offsets(base_pose, z_off_m=0.0, fwd_off_m=0.0):
        R = quaternion_to_rotation_matrix(base_pose.orientation)
        fwd = R[:, 0]
        p = np.array([base_pose.position.x_val,
                      base_pose.position.y_val,
                      base_pose.position.z_val], dtype=float)
        p = p + fwd_off_m * fwd
        p[2] = p[2] + z_off_m
        return airsim.Pose(
            position_val=airsim.Vector3r(p[0], p[1], p[2]),
            orientation_val=base_pose.orientation
        )
    @staticmethod
    def amodal_bbox_for_actor_with_dims(pose, cam_pose, P, img_w, img_h,
                                        L, W, H, z_off, fwd_off=0.0):
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
    @staticmethod
    def process_target(target_cfg, client, cam_info, cam_pose, img, seg_img, depth_img, P):

        amodal_box = None
        tight_box  = None

        label = target_cfg["label"]
        pattern = target_cfg["ACTOR_PATTERN"]
        obj_type = target_cfg["OBJECT_TYPE"]
        box_color = target_cfg["BOX_COLOR_BGR"]
        show_roi = SHOW_ROI_WINDOWS_GLOBAL and target_cfg.get("SHOW_ROI_WINDOWS", False)

        profile = PROFILES[obj_type].copy() if obj_type in PROFILES else PROFILES["car"].copy()
        if "PROFILE_OVERRIDE" in target_cfg and target_cfg["PROFILE_OVERRIDE"]:
            profile.update(target_cfg["PROFILE_OVERRIDE"])

        L, W, H = profile["L"], profile["W"], profile["H"]
        z_off   = profile["Z"]

        objs = client.simListSceneObjects(pattern)
        if not objs:
            print(f"[{label}] No actor matches '{pattern}'")
            return {"found": False}
        actor = objs[0]
        print(f"[{label}] Target actor: {actor}")

        actor_pose = _safe_get_pose(client, actor)

        fwd_off = profile.get("FWD_OFF_M", 0.0)
        adjusted_actor_pose = pose_with_offsets(actor_pose, z_off_m=z_off, fwd_off_m=fwd_off)

        corners_w = compute_bounding_box_corners_world(adjusted_actor_pose, L, W, H)

        h, w = img.shape[:2]
        disp_bbox = None
        disp_img  = img
        drew_tight = False

        if PROJECTION_ENABLED and cam_pose is not None:
            pts2d, depth_forward, valid = project_world_points_to_image(corners_w, cam_pose, P, w, h)

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
                    amodal_box = disp_bbox
                    if not (DOMINANT_COLOR_ONLY or DRAW_ONLY_CORRECTED_2D):
                        cv2.rectangle(disp_img, (x0, y0), (x1, y1), box_color, 2)
            if disp_bbox is None:
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
                        amodal_box = disp_bbox
                        if not (DOMINANT_COLOR_ONLY or DRAW_ONLY_CORRECTED_2D):
                            cv2.rectangle(disp_img, (x0, y0), (x1, y1), box_color, 2)

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

                        if Hercules2D3DDetector.SHOW_VISUALS:
                            cv2.namedWindow(f"{label} Seg ROI (depth <= {DEPTH_CLIP_MAX_M:.1f} m)", cv2.WINDOW_NORMAL)
                            cv2.imshow(f"{label} Seg ROI (depth <= {DEPTH_CLIP_MAX_M:.1f} m)", seg_roi_clipped)
                            cv2.namedWindow(f"{label} Depth ROI", cv2.WINDOW_NORMAL)
                            cv2.imshow(f"{label} Depth ROI", depth_vis)
                else:
                    seg_roi_clipped = seg_roi

                if roi_world_pts is not None and roi_colors_rgb is not None:
                    top_colors, n_inside = dominant_colors_in_box(
                        roi_world_pts, roi_colors_rgb,
                        adjusted_actor_pose, L, W, H, top_k=3
                    )
                    print(f"[{label}] Points inside 3D box: {n_inside}")

                    if DOMINANT_COLOR_ONLY and n_inside == 0:
                        print(f"[{label}] Occluded (no colored ROI points inside cuboid). Ignoring object.")
                        return {"found": False}

                    if n_inside > 0 and len(top_colors) > 0:
                        best_rgb, best_count, best_frac = top_colors[0]
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
                                # keep the exact tight rect we drew
                                tight_box = (tx0, ty0, tx1, ty1)
                                drew_tight = True
                        else:
                            print(f"[{label}] Dominant color present but tight 2D box not found (min_pixels={REFIT_MIN_PIXELS}).")
                    else:
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
            "drew_tight": drew_tight,
            "amodal_bbox_xyxy": amodal_box,
            "tight_bbox_xyxy":  tight_box,
        }
    

    @staticmethod
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
    @staticmethod
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
    @staticmethod
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

            pose = _safe_get_pose(client, idname)
            if pose is None:
                continue

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
            if (x1 - x0) * (y1 - y0) < 9:
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

            prof_override = get_profile_for_idname(idname, obj_type)

            targets.append({
                "label": label if label else idname,
                "ACTOR_PATTERN": pattern,
                "OBJECT_TYPE": obj_type,
                "BOX_COLOR_BGR": color_bgr,
                "SHOW_ROI_WINDOWS": False,
                "ENABLE_TIGHT_REFIT": True,
                "PROFILE_OVERRIDE": prof_override,
            })
        return targets
    @staticmethod
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
        # cx1, cy1 = x0 + t1 * dx, y1 + t1 * dy
        cx1, cy1 = x0 + t1 * dx, y0 + t1 * dy
        return (cx0, cy0), (cx1, cy1)

    # --- full-frame segmentation color pruning (after depth-clip) ---
    @staticmethod
    def prune_small_seg_colors(seg_img_bgr, min_pixels):
        out = np.zeros_like(seg_img_bgr)
        flat = seg_img_bgr.reshape(-1, 3)
        nz = np.any(flat != 0, axis=1)
        if not np.any(nz):
            return out
        colors, counts = np.unique(flat[nz], axis=0, return_counts=True)
        keep = colors[counts >= int(min_pixels)]
        if keep.size == 0:
            return out
        for c in keep:
            m = (seg_img_bgr[:,:,0] == c[0]) & (seg_img_bgr[:,:,1] == c[1]) & (seg_img_bgr[:,:,2] == c[2])
            out[m] = c
        return out


    def _write_dair_lite_labels(self, frame_id, results, cam_dir, lidar_dir):
        os.makedirs(cam_dir,  exist_ok=True)
        os.makedirs(lidar_dir, exist_ok=True)

        cam_out   = []
        lidar_out = []

        # cached transforms from run()
        R_sw = getattr(self, "_R_sw", None)   # world->sensor (LiDAR) in AirSim basis
        t_ws = getattr(self, "_t_ws", None)   # LiDAR origin in world (AirSim basis)
        cam_pose = getattr(self, "_cam_pose", None)

        for r in results:
            if not r.get("found"):
                continue

            # keep camera & lidar sets identical if you require LiDAR points
            lidar_pts = int(r.get("lidar_points_inside_n", 0))
            min_pts   = int(getattr(self, "MIN_LIDAR_POINTS_IN_BOX", 1))
            if getattr(self, "CAMERA_LABEL_REQUIRE_LIDAR_POINTS", False) and lidar_pts < min_pts:
                continue
            if getattr(self, "LIDAR_LABEL_REQUIRE_POINTS", False) and lidar_pts < min_pts:
                continue

            # class mapping
            obj_type  = Hercules2D3DDetector.infer_object_type_from_label(r.get("label",""))
            dair_type = "Pedestrian" if obj_type == "human" else "Car"

            # 2D bbox
            box2d = r.get("final_camera_bbox_xyxy")
            if box2d is None:
                xmin = ymin = xmax = ymax = -1.0
            else:
                xmin, ymin, xmax, ymax = [float(v) for v in box2d]

            # sizes (profile) — DAIR uses h,w,l
            L = float(r["L"]); W = float(r["W"]); H = float(r["H"])

            # world center of the oriented box
            ap  = r["adjusted_pose"]
            c_w = np.array([ap.position.x_val,
                            ap.position.y_val,
                            ap.position.z_val], dtype=float)

            # ---------------- CAMERA JSON (location in camera frame) ----------------
            if cam_pose is not None:
                R_cam = Hercules2D3DDetector.quaternion_to_rotation_matrix(cam_pose.orientation)  # camera->world
                cam_p = np.array([cam_pose.position.x_val,
                                cam_pose.position.y_val,
                                cam_pose.position.z_val], dtype=float)

                c_c = R_cam.T @ (c_w - cam_p)  # world -> camera

                # rotation_y in camera coords: forward axis of the box in camera frame
                R_box   = Hercules2D3DDetector.quaternion_to_rotation_matrix(ap.orientation)  # body->world
                fwd_cam = R_cam.T @ (R_box @ np.array([1.0, 0.0, 0.0], dtype=float))          # body x in camera
                rot_cam = float(math.atan2(fwd_cam[2], fwd_cam[0]))
            else:
                c_c = c_w
                rot_cam = 0.0

            cam_rec = {
                "type": dair_type,
                "truncated_state": "0",
                "occluded_state": "0",
                "alpha": "0",
                "2d_box": {"xmin": xmin, "ymin": ymin, "xmax": xmax, "ymax": ymax},
                "3d_dimensions": {"h": H, "w": W, "l": L},
                "3d_location": {  # CAMERA json in DAIR is often strings
                    "x": f"{c_c[0]:.6f}", "y": f"{c_c[1]:.6f}", "z": f"{c_c[2]:.6f}"
                },
                "rotation": rot_cam
            }
            cam_out.append(cam_rec)

            # ---------------- LiDAR JSON (convert AirSim -> DAIR LiDAR) -------------
            if R_sw is not None and t_ws is not None:
                # center in LiDAR sensor frame (AirSim basis: x fwd, y right, z down)
                c_s_as = R_sw @ (c_w - t_ws)

                # orientation of box in LiDAR sensor frame (AirSim basis)
                R_wb   = Hercules2D3DDetector.quaternion_to_rotation_matrix(ap.orientation)  # body->world
                R_sb   = R_sw @ R_wb                                                         # body->sensor
                yaw_as = float(math.atan2(R_sb[1, 0], R_sb[0, 0]))                            # yaw about +Z (AirSim)

                # ---- AirSim -> DAIR/KITTI LiDAR basis ----
                # y' = -y, z' = -z  (mirror about X), yaw' = -yaw
                c_s = np.array([ c_s_as[0], -c_s_as[1], -c_s_as[2] ], dtype=float)
                yaw = -yaw_as
            else:
                c_s  = c_w
                yaw  = 0.0

            lidar_rec = {
                "type": dair_type,
                "truncated_state": "0",
                "occluded_state": "0",
                "alpha": "0",
                "2d_box": {"xmin": xmin, "ymin": ymin, "xmax": xmax, "ymax": ymax},
                "3d_dimensions": {"h": H, "w": W, "l": L},   # unchanged
                "3d_location": { "x": float(c_s[0]), "y": float(c_s[1]), "z": float(c_s[2]) },
                "rotation": yaw
            }
            lidar_out.append(lidar_rec)

        with open(os.path.join(cam_dir,   f"{frame_id}.json"), "w") as f:
            json.dump(cam_out, f, indent=2)
        with open(os.path.join(lidar_dir, f"{frame_id}.json"), "w") as f:
            json.dump(lidar_out, f, indent=2)

        print(f"[label] wrote {len(cam_out)} camera and {len(lidar_out)} lidar labels for frame {frame_id}")


    # ===================== main =====================
    def run(self):
        np.set_printoptions(precision=4, suppress=True)
        client = self.CLIENT_CLASS(port=self.PORT)
        client.confirmConnection()
        print("Connected!\n")

        results_lidar_only = []

        # Zero lens distortion if any
        dparams = client.simGetDistortionParams(self.CAMERA_NAME, vehicle_name=self.VEHICLE_NAME)
        print("Distortion params:", dparams)
        if any(abs(d)>1e-9 for d in dparams):
            print(" Zeroing distortion.")
            client.simSetDistortionParams(
                self.CAMERA_NAME,
                {"K1":0.0, "K2":0.0, "K3":0.0, "P1":0.0, "P2":0.0},
                vehicle_name=self.VEHICLE_NAME
            )
        else:
            print(" No distortion active.")
        print()

        # ===================== SINGLE PAUSE WINDOW =====================
        client.simPause(True)

        try:
            # (1) Camera info and synchronized image pack
            cam_info = client.simGetCameraInfo(self.CAMERA_NAME, vehicle_name=self.VEHICLE_NAME)
            cam_pose = cam_info.pose if cam_info else None

            # Make camera pose "true world": add per-vehicle spawn translation.
            try:
                if cam_pose is not None:
                    spawn_xyz = Hercules2D3DDetector.load_vehicle_spawn_translation(
                        Hercules2D3DDetector.SETTINGS_JSON_PATH,
                        self.VEHICLE_NAME
                    )
                    cam_pose = Hercules2D3DDetector.pose_add_translation(cam_pose, spawn_xyz)
                    self._cam_pose = cam_pose

            except Exception as e:
                print(f"[WARN] camera spawn translation not applied: {e}")

            # IMPORTANT: pass vehicle_name on the simGetImages() call, not per request.
            reqs = [
                airsim.ImageRequest(self.CAMERA_NAME, airsim.ImageType.Scene,        False, True),
                airsim.ImageRequest(self.CAMERA_NAME, airsim.ImageType.Segmentation, False, True),
                airsim.ImageRequest(self.CAMERA_NAME, airsim.ImageType.DepthPerspective, True, False),
            ]
            scene_resp, seg_resp, depth_resp = client.simGetImages(reqs, vehicle_name=self.VEHICLE_NAME)

            print("IMAGE DATA TS: ", scene_resp.time_stamp, seg_resp.time_stamp, depth_resp.time_stamp)

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

            # (1b) LiDAR data — sampled inside the SAME pause window  === LiDAR: ===
            lidar_pcd_world = None
            lidar_pcd_sensor = None     # for the second window
            lidar_frame_mesh = None
            # transforms we'll reuse for sensor-frame window
            R_wv = None; t_wv = None; R_vs = None; t_vs = None; R_ws = None; t_ws = None; R_sw = None

            if o3d is not None and self.ADD_LIDAR_TO_OPEN3D:
                lidar_data = None
                try:
                    # Newer signature: (lidar_name, vehicle_name)
                    lidar_data = client.getLidarData(self.LIDAR_NAME, self.VEHICLE_NAME)
                    print("LIDAR DATA TS: ", lidar_data.time_stamp)
                    print("TIME DIFFERENCE: ", (scene_resp.time_stamp - lidar_data.time_stamp)/(10**9))
                except TypeError:
                    # Older signature may omit vehicle name
                    lidar_data = client.getLidarData(self.LIDAR_NAME)
                except Exception as e:
                    print("LiDAR fetch error:", e)
                    lidar_data = None

                if lidar_data is not None and hasattr(lidar_data, "point_cloud") and len(lidar_data.point_cloud) >= 3:
                    pts = np.array(lidar_data.point_cloud, dtype=np.float64).reshape(-1, 3)
                    if self.LIDAR_STRIDE > 1:
                        pts = pts[::self.LIDAR_STRIDE]

                    # Compose world transform: world <- vehicle <- sensor
                    # Get pose in a way that ALWAYS respects vehicle_name
                    veh_pose = None
                    try:
                        if hasattr(client, "getCarState"):
                            st = client.getCarState(vehicle_name=self.VEHICLE_NAME)
                            veh_pose = airsim.Pose(st.kinematics_estimated.position,
                                                    st.kinematics_estimated.orientation)
                        elif hasattr(client, "getMultirotorState"):
                            st = client.getMultirotorState(vehicle_name=self.VEHICLE_NAME)
                            veh_pose = airsim.Pose(st.kinematics_estimated.position,
                                                    st.kinematics_estimated.orientation)
                    except Exception:
                        pass
                    if veh_pose is None:
                        try:
                            # newer RPCs take vehicle_name as kwarg
                            veh_pose = client.simGetVehiclePose(vehicle_name=self.VEHICLE_NAME)
                        except TypeError:
                            # last resort: may return "default vehicle" on some builds
                            veh_pose = client.simGetVehiclePose()

                    # --- apply world spawn translation from settings.json ---
                    try:
                        spawn_xyz = Hercules2D3DDetector.load_vehicle_spawn_translation(
                            Hercules2D3DDetector.SETTINGS_JSON_PATH,
                            self.VEHICLE_NAME
                        )
                        veh_pose = Hercules2D3DDetector.pose_add_translation(veh_pose, spawn_xyz)
                    except Exception as e:
                        print(f"[WARN] spawn translation not applied: {e}")

                    R_vs = quaternion_to_rotation_matrix(lidar_data.pose.orientation)
                    t_vs = np.array([lidar_data.pose.position.x_val,
                                    lidar_data.pose.position.y_val,
                                    lidar_data.pose.position.z_val], dtype=float)
                    R_wv = quaternion_to_rotation_matrix(veh_pose.orientation)
                    t_wv = np.array([veh_pose.position.x_val,
                                    veh_pose.position.y_val,
                                    veh_pose.position.z_val], dtype=float)

                    # World points
                    p_world = (R_wv @ (R_vs @ pts.T + t_vs.reshape(3,1))).T + t_wv

                    # Build Open3D point cloud (world)
                    lidar_pcd_world = o3d.geometry.PointCloud()
                    lidar_pcd_world.points = o3d.utility.Vector3dVector(p_world)
                    if self.LIDAR_VOXEL_DOWNSAMPLE and self.LIDAR_VOXEL_DOWNSAMPLE > 0:
                        lidar_pcd_world = lidar_pcd_world.voxel_down_sample(self.LIDAR_VOXEL_DOWNSAMPLE)
                    lidar_pcd_world.paint_uniform_color(self.LIDAR_COLOR_RGB)
                    print(f"LiDAR: {np.asarray(lidar_pcd_world.points).shape[0]} point(s) @ ts={getattr(lidar_data, 'time_stamp', 'NA')}.")

                    # LiDAR pose in world, and world->sensor transform
                    R_ws = R_wv @ R_vs
                    t_ws = t_wv + (R_wv @ t_vs)
                    R_sw = R_ws.T

                    self._R_sw = R_sw
                    self._t_ws = t_ws

                    lidar_cfg = Hercules2D3DDetector.load_lidar_fov_from_settings(
                        Hercules2D3DDetector.SETTINGS_JSON_PATH,
                        Hercules2D3DDetector.VEHICLE_NAME,
                        Hercules2D3DDetector.LIDAR_NAME
                    )
                    print("LiDAR FOV/Range from settings:", lidar_cfg)

                    # === LiDAR SENSOR-FRAME WINDOW (NEW) ===
                    p_sensor = (R_sw @ (p_world.T - t_ws.reshape(3,1))).T
                    lidar_pcd_sensor = o3d.geometry.PointCloud()
                    lidar_pcd_sensor.points = o3d.utility.Vector3dVector(p_sensor)
                    if self.LIDAR_VOXEL_DOWNSAMPLE and self.LIDAR_VOXEL_DOWNSAMPLE > 0:
                        lidar_pcd_sensor = lidar_pcd_sensor.voxel_down_sample(self.LIDAR_VOXEL_DOWNSAMPLE)
                    lidar_pcd_sensor.paint_uniform_color(self.LIDAR_COLOR_RGB)

                    # Optional: sensor frame in world window (for reference)
                    T_ws = np.eye(4)
                    T_ws[:3,:3] = R_ws
                    T_ws[:3, 3] = t_ws
                    lidar_frame_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
                    lidar_frame_mesh.transform(T_ws)
                else:
                    print("LiDAR: no points.")
            # === end LiDAR: ===

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

                if Hercules2D3DDetector.SHOW_VISUALS:
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

            # (4) Force-include the requested ID(s)
            for idname_req in FORCE_INCLUDE_IDNAMES:
                resolved = resolve_id_exact_or_prefix(client, idname_req)
                if not resolved:
                    continue
                already = any(re.fullmatch(t["ACTOR_PATTERN"].strip("^$"), resolved) for t in TARGETS)
                if already:
                    print(f"[force] '{resolved}' already in TARGETS from FOV pass.")
                    continue
                obj_type = FORCE_INCLUDE_OBJECT_TYPE or infer_object_type_from_label(
                    id_to_label.get(resolved, resolved)
                )
                color_bgr = FORCE_INCLUDE_COLOR_BGR or BOX_COLORS_BGR[len(TARGETS) % len(BOX_COLORS_BGR)]
                prof_override = get_profile_for_idname(resolved, obj_type)
                TARGETS.append({
                    "label": id_to_label.get(resolved, resolved),
                    "ACTOR_PATTERN": f"^{re.escape(resolved)}$",
                    "OBJECT_TYPE": obj_type,
                    "BOX_COLOR_BGR": color_bgr,
                    "SHOW_ROI_WINDOWS": False,
                    "ENABLE_TIGHT_REFIT": True,
                    "PROFILE_OVERRIDE": prof_override,
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

            # After building id_to_label and camera-based TARGETS/results
            actor_names_from_camera = {r["actor_name"] for r in results if r.get("found", False)}

            if (R_sw is not None) and (t_ws is not None) and ('lidar_cfg' in locals()):
                results_lidar_all = Hercules2D3DDetector.build_lidar_targets(
                    client, id_to_label, R_sw, t_ws, lidar_cfg
                )
                # Keep only those NOT already in the camera-based set (to avoid duplicate boxes)
                results_lidar_only = [r for r in results_lidar_all if r["actor_name"] not in actor_names_from_camera]
            else:
                results_lidar_only = []

            # Compute LiDAR points-in-box counts for the CAMERA-selected objects
            if 'p_world' in locals() and p_world is not None:
                for r in results:
                    if not r.get("found", False):
                        continue
                    inside_mask = Hercules2D3DDetector.points_inside_oriented_box(
                        p_world, r["adjusted_pose"], r["L"], r["W"], r["H"]
                    )
                    r["lidar_points_inside_n"] = int(inside_mask.sum())
            else:
                for r in results:
                    if r.get("found", False):
                        r["lidar_points_inside_n"] = 0

        finally:
            # ===================== END SINGLE PAUSE WINDOW =====================
            client.simPause(False)

        # ============ (1c) Decide & store the EXACT 2D box that was shown ============
        # This ensures the label writer can save the same rectangle the UI displayed.
        for res in results:
            if not res.get("found", False):
                continue
            if (DRAW_ONLY_CORRECTED_2D or DOMINANT_COLOR_ONLY):
                # UI policy: show tight boxes only (skip if none found)
                final_2d = res.get("tight_bbox_xyxy")
                final_src = "tight" if final_2d is not None else None
            else:
                # UI policy: show tight when available, else fall back to amodal
                final_2d = res.get("tight_bbox_xyxy") or res.get("amodal_bbox_xyxy")
                final_src = "tight" if res.get("tight_bbox_xyxy") is not None else ("amodal" if final_2d is not None else None)

            res["final_camera_bbox_xyxy"] = final_2d
            res["final_camera_box_type"]  = final_src
            if final_src is None:
                print(f"[{res.get('label','?')}] No 2D box to save under current UI policy.")
            else:
                x0,y0,x1,y1 = final_2d
                print(f"[{res.get('label','?')}] final 2D box ({final_src}) = ({x0},{y0})–({x1},{y1})")

            # Combine camera-selected and LiDAR-FOV-only results for labeling
            results_all = results + results_lidar_only

            # Count LiDAR points inside each 3D cuboid (for gating)
            if 'p_world' in locals() and p_world is not None:
                for r in results_all:
                    if not r.get("found"): 
                        continue
                    inside = Hercules2D3DDetector.points_inside_oriented_box(
                        p_world, r["adjusted_pose"], r["L"], r["W"], r["H"]
                    )
                    r["lidar_points_inside_n"] = int(inside.sum())
            else:
                for r in results_all:
                    r["lidar_points_inside_n"] = 0

            # Now write BOTH 2D and 3D labels for the combined set
            if self.SAVE_LABELS and self.FRAME_ID and self.LABEL_CAMERA_DIR and self.LABEL_LIDAR_DIR:
                self._write_dair_lite_labels(self.FRAME_ID, results_all, self.LABEL_CAMERA_DIR, self.LABEL_LIDAR_DIR)

        # (6) 2D annotation & legend (after unpause, using frozen results)
        cv2.putText(disp, f"Camera: {self.CAMERA_NAME}", (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0), 2)
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
        
        if Hercules2D3DDetector.SHOW_VISUALS:
            cv2.namedWindow(title, cv2.WINDOW_NORMAL)
            cv2.imshow(title, disp)
            print("Press any key to exit (after Open3D closes if opened).")
            cv2.waitKey(1)

        # (7) Open3D viz using the same frozen corners/pcds captured while PAUSED
        if o3d:
            try:
                # ================= WORLD-FRAME WINDOW =================
                geoms = []

                # camera frame (optional)
                Tc = np.eye(4)
                Tc[:3, :3] = quaternion_to_rotation_matrix(cam_pose.orientation)
                Tc[:3, 3]  = [cam_pose.position.x_val, cam_pose.position.y_val, cam_pose.position.z_val]
                fc = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
                fc.transform(Tc)
                geoms.append(fc)

                # LiDAR frame and cloud in world
                if 'lidar_frame_mesh' in locals() and lidar_frame_mesh is not None:
                    geoms.append(lidar_frame_mesh)
                if 'lidar_pcd_world' in locals() and lidar_pcd_world is not None:
                    geoms.append(lidar_pcd_world)

                # Add ROI point clouds (only for kept objects)
                for res in results:
                    if res.get("found", False) and res["roi_pcd"] is not None:
                        geoms.append(res["roi_pcd"])

                # Add cuboid line sets LAST so they render on top
                edges = [[0,1],[1,3],[3,2],[2,0],[4,5],[5,7],[7,6],[6,4],[0,4],[1,5],[2,6],[3,7]]
                for res in (results + results_lidar_only):
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
                    print("Showing 3D viz in Open3D (world frame).")
                    if Hercules2D3DDetector.SHOW_VISUALS:
                        vis = o3d.visualization.Visualizer()
                        vis.create_window(window_name="Open3D: World (LiDAR + ROI + 3D Boxes)")
                        for g in geoms:
                            vis.add_geometry(g)
                        opt = vis.get_render_option()
                        opt.background_color = np.asarray([1.0, 1.0, 1.0])  # white background
                        opt.point_size = 5.0
                        if hasattr(opt, "line_width"):
                            opt.line_width = 3.0
                        vis.run()
                        vis.destroy_window()
                else:
                    print("Open3D: nothing to show in world frame.")

                # ============ LiDAR SENSOR-FRAME WINDOW (NEW) ============
                if ('lidar_pcd_sensor' in locals() and lidar_pcd_sensor is not None
                        and R_sw is not None and t_ws is not None):
                    geoms_lidar = []
                    cf_lidar = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
                    geoms_lidar.append(cf_lidar)
                    geoms_lidar.append(lidar_pcd_sensor)

                    # define edges here so this block is self-contained
                    edges = [[0,1],[1,3],[3,2],[2,0],[4,5],[5,7],[7,6],[6,4],[0,4],[1,5],[2,6],[3,7]]

                    # draw both camera-selected and LiDAR-FOV-only boxes
                    for res in (results + results_lidar_only):
                        if not res.get("found", False):
                            continue

                        # Drop boxes that have zero LiDAR points inside
                        if 'p_world' in locals() and p_world is not None:
                            inside = Hercules2D3DDetector.points_inside_oriented_box(
                                p_world, res["adjusted_pose"], res["L"], res["W"], res["H"]
                            )
                            if int(inside.sum()) == 0:
                                continue

                        # transform and draw the remaining box in LiDAR (sensor) frame
                        corners_w = res["corners_w"]
                        corners_s = (R_sw @ (corners_w - t_ws).T).T
                        bgr = res["box_color"]
                        rgb = [bgr[2]/255.0, bgr[1]/255.0, bgr[0]/255.0]
                        ls_s = o3d.geometry.LineSet(
                            points=o3d.utility.Vector3dVector(corners_s),
                            lines=o3d.utility.Vector2iVector(edges)
                        )
                        ls_s.colors = o3d.utility.Vector3dVector([rgb] * len(edges))
                        geoms_lidar.append(ls_s)

                    print("Showing 3D viz in Open3D (LiDAR sensor frame).")
                    if Hercules2D3DDetector.SHOW_VISUALS:
                        vis2 = o3d.visualization.Visualizer()
                        vis2.create_window(window_name="Open3D: LiDAR frame (sensor-local)")
                        for g in geoms_lidar:
                            vis2.add_geometry(g)
                        opt2 = vis2.get_render_option()
                        opt2.background_color = np.asarray([1.0, 1.0, 1.0])  # white background
                        opt2.point_size = 5.0
                        if hasattr(opt2, "line_width"):
                            opt2.line_width = 3.0
                        vis2.run()
                        vis2.destroy_window()
                else:
                    print("LiDAR sensor-frame window skipped (missing cloud or transforms).")
            except Exception as e:
                print("Open3D error:", e)

        cv2.waitKey(0)
        cv2.destroyAllWindows()


# --- Compatibility shims to preserve exact functionality ---
_CONFIG_NAMES = []
for _name, _val in vars(Hercules2D3DDetector).items():
    if _name.isupper():
        globals()[_name] = _val
        _CONFIG_NAMES.append(_name)

_HELPER_NAMES = [
    'load_actor_map',
    'label_has_keyword',
    'infer_object_type_from_label',
    'cam_to_point_range',
    'quaternion_to_euler',
    'quaternion_to_rotation_matrix',
    'print_pose',
    'compute_intrinsics_from_horizontal_fov',
    'compute_bounding_box_corners_world',
    'project_world_points_to_image',
    'draw_2d_bbox_and_get_rect',
    'resize_to',
    'depth_roi_to_vis',
    'roi_points_to_world_pointcloud_P',
    'points_inside_oriented_box',
    'dominant_colors_in_box',
    'tight_box_for_color',
    'world_to_cam_and_pixel',
    'approx_visible',
    '_safe_get_pose',
    'resolve_id_exact_or_prefix',
    'get_profile_for_idname',
    'pose_with_offsets',
    'amodal_bbox_for_actor_with_dims',
    'process_target',
    'obb_from_points',
    'amodal_bbox_for_actor',
    'build_targets_from_csv_scene',
    '_clip_segment_to_rect',
    'prune_small_seg_colors'
]
for _fname in _HELPER_NAMES:
    globals()[_fname] = getattr(Hercules2D3DDetector, _fname)

# Note: No module-level main(). Use Hercules2D3DDetector().run() instead.
