#!/usr/bin/env python3
import setup_path
import hercules as airsim
import cv2, numpy as np, math

# ---------------- USER CONFIG ----------------
VEHICLE = "Husky1"
CAM     = "front_center"
PORT    = 41452
IMG_T   = airsim.ImageType.Scene

MAX_DIST_M       = 200.0
DISPLAY_W, DISPLAY_H = 1920, 1080
USE_OCCLUSION    = False          # turn on after geometry is correct
DEBUG_DUMP_ONCE  = True           # print one big block
PRINT_FIRST_N    = 1              # how many dets to dump in that block

MESH_PATTERNS = [
    "BP_VehicleAI*","Sportscar*","Sedan*","Van1*","Copcar*",
    "Hugetruck*","Sedan1*","SUV1*","Milkvan*","Sedan2*",
    "Pickuptruck*","Garbagetruck*","BP_SplineHuman*"
]

# ---------------- CONNECT ----------------
client = airsim.CarClient(ip="127.0.0.1", port=PORT); client.confirmConnection()
client.simSetDetectionFilterRadius(CAM, IMG_T, int(MAX_DIST_M*100))  # cm
client.simClearDetectionMeshNames(CAM, IMG_T)
for pat in MESH_PATTERNS:
    client.simAddDetectionFilterMeshName(CAM, IMG_T, pat)

# ---------------- MATH ----------------
def fx_fy(w,h,fov_deg):
    fx = (w/2)/math.tan(math.radians(fov_deg)/2)
    fy = (h/2)/math.tan(math.radians(fov_deg)/2)
    return fx, fy

def quat_to_rot(w,x,y,z):
    s=2.0; xx,yy,zz=x*x,y*y,z*z; xy,xz,yz=x*y,x*z,y*z; wx,wy,wz=w*x,w*y,w*z
    return np.array([
        [1-s*(yy+zz),   s*(xy-wz),   s*(xz+wy)],
        [  s*(xy+wz), 1-s*(xx+zz),   s*(yz-wx)],
        [  s*(xz-wy),   s*(yz+wx), 1-s*(xx+yy)]
    ], dtype=np.float32)

def get_world_to_cam():
    info = client.simGetCameraInfo(CAM, vehicle_name=VEHICLE)
    q,p = info.pose.orientation, info.pose.position  # AirSim: NED, meters
    R_c2w = quat_to_rot(q.w_val,q.x_val,q.y_val,q.z_val)
    t_c2w = np.array([p.x_val,p.y_val,p.z_val], np.float32)
    R_w2c = R_c2w.T
    t_w2c = -R_w2c @ t_c2w
    return R_w2c, t_w2c, info.fov

# UE Z-up -> NED Z-down, cm->m
def ue_to_ned(P_cm):
    P_m = P_cm/100.0
    P_m[:,2] *= -1.0
    return P_m

# candidate OpenCV mappings: (x_cv, y_cv, z_cv)
CANDIDATES = {
    "map1":  lambda Pc: np.stack([ Pc[:,1],  Pc[:,2],  Pc[:,0]], axis=1),   # +y, +z, +x
    "map2":  lambda Pc: np.stack([ Pc[:,1], -Pc[:,2],  Pc[:,0]], axis=1),   # +y, -z, +x  (flip vertical)
    "map3":  lambda Pc: np.stack([-Pc[:,1], Pc[:,2],  Pc[:,0]], axis=1),    # -y, +z, +x
    "map4":  lambda Pc: np.stack([ Pc[:,0],  Pc[:,1],  Pc[:,2]], axis=1),   # sanity check
}

EDGES = [(0,1),(1,2),(2,3),(3,0),
         (4,5),(5,6),(6,7),(7,4),
         (0,4),(1,5),(2,6),(3,7)]

def corners_from_box3d(b):
    mn,mx = b.min, b.max
    return np.array([
        [mn.x_val,mn.y_val,mn.z_val],
        [mx.x_val,mn.y_val,mn.z_val],
        [mx.x_val,mx.y_val,mn.z_val],
        [mn.x_val,mx.y_val,mn.z_val],
        [mn.x_val,mn.y_val,mx.z_val],
        [mx.x_val,mn.y_val,mx.z_val],
        [mx.x_val,mx.y_val,mx.z_val],
        [mn.x_val,mx.y_val,mx.z_val],
    ], dtype=np.float32)

def oriented_corners_from_pose(det):
    raw = corners_from_box3d(det.box3D)
    ex,ey,ez = (raw.max(0) - raw.min(0))/200.0  # cm->m, half
    pose = client.simGetObjectPose(det.name)
    if not pose or (pose.position.x_val==0 and pose.position.y_val==0 and pose.position.z_val==0):
        return None
    center = np.array([pose.position.x_val,
                       pose.position.y_val,
                       pose.position.z_val], np.float32)
    q = pose.orientation
    R_o2w = quat_to_rot(q.w_val,q.x_val,q.y_val,q.z_val)
    signs = np.array([[ 1, 1, 1],
                      [ 1, 1,-1],
                      [ 1,-1, 1],
                      [ 1,-1,-1],
                      [-1, 1, 1],
                      [-1, 1,-1],
                      [-1,-1, 1],
                      [-1,-1,-1]], np.float32)
    local = signs * np.array([ex,ey,ez], np.float32)
    Pw = (R_o2w @ local.T).T + center
    return Pw  # NED world, m

def project(Pc_cv, fx, fy, cx, cy):
    x,y,z = Pc_cv
    if z <= 1e-6: return None
    return int(fx*x/z + cx), int(fy*y/z + cy)

def draw_box(img, pts2, color=(0,255,0)):
    for i,j in EDGES:
        if pts2[i] and pts2[j]:
            cv2.line(img, pts2[i], pts2[j], color, 2)
    for p in pts2:
        if p: cv2.circle(img, p, 3, (0,255,255), -1)

# -------- OCCLUSION (optional) --------
def get_seg_gray():
    seg_raw = client.simGetImage(CAM, airsim.ImageType.Segmentation)
    if not seg_raw: return None
    seg = cv2.imdecode(airsim.string_to_uint8_array(seg_raw), cv2.IMREAD_COLOR)
    return None if seg is None else seg[:,:,0]

obj_id_cache = {}
def visible_ratio(seg_gray, box, oid):
    if seg_gray is None or oid < 0: return 1.0
    x1,y1,x2,y2 = box
    x1,y1 = max(0,x1), max(0,y1)
    x2,y2 = min(seg_gray.shape[1],x2), min(seg_gray.shape[0],y2)
    if x2<=x1 or y2<=y1: return 0.0
    crop = seg_gray[y1:y2, x1:x2]
    return float(np.mean(crop == oid))

# -------- DEBUG BLOCK --------
np.set_printoptions(suppress=True, linewidth=120)

def dump_det_debug(det, R_w2c, t_w2c, fx, fy, cx, cy, w, h):
    raw_cm = corners_from_box3d(det.box3D)
    Pw_ned = ue_to_ned(raw_cm)             # cm->m + Z flip
    Pc_ned = (R_w2c @ Pw_ned.T + t_w2c[:,None]).T

    print("\n==== DEBUG DUMP ====")
    print("name:", det.name)
    print("raw_cm (UE):\n", raw_cm)
    print("Pw_ned (m):\n", Pw_ned[:3])
    print("Pc_ned (first 3):\n", Pc_ned[:3])
    pose = client.simGetObjectPose(det.name)
    print("obj pose m:", pose.position)
    print("obj quat:", pose.orientation)
    print("R_w2c:\n", R_w2c)
    print("t_w2c:", t_w2c)

    for key, fn in CANDIDATES.items():
        Pc_cv = fn(Pc_ned)
        pts2  = [project(p, fx, fy, cx, cy) for p in Pc_cv]
        inside = sum(1 for p in pts2 if p and 0<=p[0]<w and 0<=p[1]<h)
        mean_z = float(np.mean(Pc_cv[:,2]))
        print(f"{key}: inside={inside}, mean_z={mean_z:.3f}, sample={pts2[:3]}")
    print("==== END DEBUG ====\n")

# -------- PICK BEST MAPPING --------
def pick_mapping(Pc_ned, w, h, fx, fy, cx, cy):
    best_key, best_inside, best_z = None, -1, -1
    best_pts, best_cv = None, None
    for key, fn in CANDIDATES.items():
        Pc_cv = fn(Pc_ned)
        pts2  = [project(p, fx, fy, cx, cy) for p in Pc_cv]
        inside = sum(1 for p in pts2 if p and 0<=p[0]<w and 0<=p[1]<h)
        mean_z = float(np.mean(Pc_cv[:,2] > 0))
        if inside > best_inside or (inside == best_inside and mean_z > best_z):
            best_key, best_inside, best_z = key, inside, mean_z
            best_pts, best_cv = pts2, Pc_cv
    return best_key, best_pts, best_cv

# ---------------- WINDOWS ----------------
cv2.namedWindow("2D", cv2.WINDOW_NORMAL)
cv2.namedWindow("3D", cv2.WINDOW_NORMAL)
cv2.resizeWindow("2D", DISPLAY_W, DISPLAY_H)
cv2.resizeWindow("3D", DISPLAY_W, DISPLAY_H)

while True:
    raw = client.simGetImage(CAM, IMG_T, vehicle_name=VEHICLE)
    if not raw: continue
    frame = cv2.imdecode(airsim.string_to_uint8_array(raw), cv2.IMREAD_COLOR)
    if frame is None: continue

    h,w = frame.shape[:2]
    R_w2c, t_w2c, fov_deg = get_world_to_cam()
    fx, fy = fx_fy(w, h, fov_deg); cx, cy = w/2.0, h/2.0

    img2d = frame.copy()
    img3d = frame.copy()

    seg_gray = get_seg_gray() if USE_OCCLUSION else None

    dets = client.simGetDetections(CAM, IMG_T, vehicle_name=VEHICLE)
    if DEBUG_DUMP_ONCE:
        print("detections:", 0 if not dets else len(dets))

    drawn2d = drawn3d = 0
    if dets:
        if DEBUG_DUMP_ONCE:
            for k,d in enumerate(dets[:PRINT_FIRST_N]):
                dump_det_debug(d, R_w2c, t_w2c, fx, fy, cx, cy, w, h)
        for d in dets:
            # ---------------- 2D ----------------
            x1,y1 = int(d.box2D.min.x_val), int(d.box2D.min.y_val)
            x2,y2 = int(d.box2D.max.x_val), int(d.box2D.max.y_val)
            if USE_OCCLUSION:
                if d.name not in obj_id_cache:
                    obj_id_cache[d.name] = client.simGetSegmentationObjectID(d.name)
                if visible_ratio(seg_gray, (x1,y1,x2,y2), obj_id_cache[d.name]) < 0.15:
                    continue
            cv2.rectangle(img2d, (x1,y1), (x2,y2), (0,0,255), 2)
            cv2.putText(img2d, d.name, (x1, y1-6), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0,0,255), 1, cv2.LINE_AA)
            drawn2d += 1

            # ---------------- 3D ----------------
            # Prefer oriented box; fall back to raw AABB
            Pw = oriented_corners_from_pose(d)
            if Pw is None:
                Pw = ue_to_ned(corners_from_box3d(d.box3D))
            Pc_ned = (R_w2c @ Pw.T + t_w2c[:,None]).T

            key, pts2, Pc_cv = pick_mapping(Pc_ned, w, h, fx, fy, cx, cy)
            inside = sum(1 for p in pts2 if p and 0<=p[0]<w and 0<=p[1]<h)
            if inside >= 2:
                draw_box(img3d, pts2, (0,255,0))
                drawn3d += 1

        if DEBUG_DUMP_ONCE:
            print(f"chosen_map (first frame): {key}, lines_drawn={drawn3d}, pts_per_box={inside}")
            DEBUG_DUMP_ONCE = False

    cv2.imshow("2D", img2d)
    cv2.imshow("3D", img3d)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
