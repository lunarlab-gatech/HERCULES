#!/usr/bin/env python3
import setup_path                   # keep if you're using the local repo copy
import hercules as airsim
import numpy as np
import csv
import cv2
import math

# ---- user params ----
PORT                    = 41451
CAMERA_NAME             = "front_center"
VEHICLE_NAME            = ""  # empty for default / single-vehicle setups

CSV_FILENAME            = "/home/sgarimella34/multi-robot-coordination/" \
                          "HERCULES/csv_data/instance_segmentation_colormap.csv"
UE_LABEL_CSV_PATH       = "/home/sgarimella34/multi-robot-coordination/" \
                          "HERCULES/csv_data/ue_label_vs_name.csv"
DEPTH_NPY_FILENAME      = "/home/sgarimella34/multi-robot-coordination/" \
                          "HERCULES/csv_data/depth_frame.npy"

FLIP_VERTICAL           = False
KEYWORDS                = ("human", "car", "truck", "sedan", "suv", "vehicle")
MAX_DISTANCE            = 30.0
IOU_THRESHOLD           = 0.1
CENTROID_DIST_THRESHOLD = 40    # pixels
ICP_MAX_ITERS           = 10
ICP_CONV_THRESH         = 1e-3
BOX_COLOR_KNOWN         = (255, 255, 255)  # white
BOX_COLOR_UNKNOWN       = (0, 255, 0)      # green
BOX_HALF                = 20    # half-size of projected box

# ---- helper functions ----
def world_to_camera(points, cam_pos, cam_orient):
    """Transform Nx3 world points into camera coordinates."""
    q = cam_orient
    Rcw = np.array([
        [1-2*(q.y_val*q.y_val+q.z_val*q.z_val),   2*(q.x_val*q.y_val- q.z_val*q.w_val),   2*(q.x_val*q.z_val+ q.y_val*q.w_val)],
        [  2*(q.x_val*q.y_val+ q.z_val*q.w_val), 1-2*(q.x_val*q.x_val+q.z_val*q.z_val),   2*(q.y_val*q.z_val- q.x_val*q.w_val)],
        [  2*(q.x_val*q.z_val- q.y_val*q.w_val),   2*(q.y_val*q.z_val+ q.x_val*q.w_val), 1-2*(q.x_val*q.x_val+q.y_val*q.y_val)]
    ])
    world_pts = np.array([[p.x_val, p.y_val, p.z_val] for p in points]).T
    rel = world_pts - np.array([[cam_pos.x_val], [cam_pos.y_val], [cam_pos.z_val]])
    return (Rcw.T.dot(rel)).T

def project_to_image(cam_pts, cam_info, width, height):
    """Project Nx3 camera coords into pixel (u,v)."""
    fov_y = math.radians(cam_info.fov)
    fy = height / (2 * math.tan(fov_y/2))
    fx = fy * (width/height)
    pixels = []
    for x_c, y_c, z_c in cam_pts:
        if z_c <= 0:
            pixels.append(None)
        else:
            u = int((fx * (x_c / z_c)) + width/2)
            v = int((fy * (-y_c / z_c)) + height/2)
            pixels.append((u, v))
    return pixels

def box_iou(a, b):
    """Compute IoU of two boxes a and b each = (xmin,ymin,xmax,ymax)."""
    xA, yA = max(a[0], b[0]), max(a[1], b[1])
    xB, yB = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, xB - xA) * max(0, yB - yA)
    areaA = (a[2] - a[0]) * (a[3] - a[1])
    areaB = (b[2] - b[0]) * (b[3] - b[1])
    union = areaA + areaB - inter
    return inter / union if union > 0 else 0

def icp_2d(source, target, max_iters=ICP_MAX_ITERS, tol=ICP_CONV_THRESH):
    """
    Perform 2D ICP between source (Nx2) and target (Mx2) point sets.
    Returns R (2x2), t (2,), and final mean error.
    """
    src = source.copy()
    R = np.eye(2)
    t = np.zeros(2)
    prev_err = float('inf')
    for _ in range(max_iters):
        # nearest neighbors
        dists = np.linalg.norm(src[:,None,:] - target[None,:,:], axis=2)
        idx   = np.argmin(dists, axis=1)
        matched = target[idx]
        mu_s = src.mean(axis=0)
        mu_t = matched.mean(axis=0)
        ds = src - mu_s
        dt = matched - mu_t
        H  = ds.T @ dt
        U, _, Vt = np.linalg.svd(H)
        R_i = Vt.T @ U.T
        if np.linalg.det(R_i) < 0:
            Vt[1,:] *= -1
            R_i = Vt.T @ U.T
        t_i = mu_t - R_i @ mu_s
        src = (R_i @ src.T).T + t_i
        R   = R_i @ R
        t   = R_i @ t + t_i
        err = np.mean(np.linalg.norm(src - matched, axis=1))
        if abs(prev_err - err) < tol:
            break
        prev_err = err
    return R, t, err

def main():
    client = airsim.MultirotorClient(port=PORT)
    client.confirmConnection()
    print("Connected!")

    # 1. dump & reload CSV (for known mapping)
    objs = client.simListInstanceSegmentationObjects()
    cmap = client.simGetSegmentationColorMap()
    with open(CSV_FILENAME, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["ObjectName","R","G","B"])
        for i, nm in enumerate(objs):
            r,g,b = map(int, cmap[i])
            wr.writerow([nm, r, g, b])
    name_to_color = {}
    with open(CSV_FILENAME, newline="") as f:
        for r in csv.DictReader(f):
            name_to_color[r["ObjectName"]] = (int(r["R"]),int(r["G"]),int(r["B"]))

    # 2. reload UE labels
    id_to_label = {}
    with open(UE_LABEL_CSV_PATH, newline="") as f:
        for r in csv.DictReader(f):
            id_to_label[r["get_name"]] = r["actor_label"]

    # 3. grab segmentation + depth
    client.simPause(True)
    seg_resp, depth_resp = client.simGetImages([
        airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.Segmentation,   False, False),
        airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.DepthPlanar,     True, False)
    ], vehicle_name=VEHICLE_NAME)
    client.simPause(False)

    img_rgb   = np.frombuffer(seg_resp.image_data_uint8, np.uint8)\
                  .reshape(seg_resp.height, seg_resp.width, 3)
    if FLIP_VERTICAL:
        img_rgb = np.flipud(img_rgb)
    depth_img = np.array(depth_resp.image_data_float, np.float32)\
                  .reshape(depth_resp.height, seg_resp.width)
    if FLIP_VERTICAL:
        depth_img = np.flipud(depth_img)
    np.save(DEPTH_NPY_FILENAME, depth_img)

    # two windows
    display_known   = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
    display_unknown = display_known.copy()
    W, H = seg_resp.width, seg_resp.height

    # 4. draw known boxes
    seg_detected = []
    for mesh, rgb in name_to_color.items():
        lbl = id_to_label.get(mesh, "").lower()
        if not lbl or not any(k in lbl for k in KEYWORDS):
            continue
        mask = ((img_rgb[:,:,0]==rgb[0]) &
                (img_rgb[:,:,1]==rgb[1]) &
                (img_rgb[:,:,2]==rgb[2]) &
                (depth_img <= MAX_DISTANCE))
        if not mask.any():
            continue
        seg_detected.append(mesh)
        m8 = (mask.astype(np.uint8)*255)
        cnts,_ = cv2.findContours(m8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        pts = np.vstack([c.reshape(-1,2) for c in cnts])
        x0,y0 = pts.min(axis=0); x1,y1 = pts.max(axis=0)
        cv2.rectangle(display_known, (x0,y0), (x1,y1), BOX_COLOR_KNOWN, 2)
        txt = id_to_label[mesh]
        if len(txt)>30: txt = txt[:27]+"..."
        cv2.putText(display_known, txt, (x0, max(0,y0-6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, BOX_COLOR_KNOWN, 1, cv2.LINE_AA)

    print(f"Segmentation-detected within {MAX_DISTANCE}m:")
    for m in seg_detected:
        print(" -", id_to_label[m])

    # 5. collect pure mask-based instances (all colors except background)
    uniq_colors = np.unique(img_rgb.reshape(-1,3), axis=0)
    unknown_bboxes = []
    for c in uniq_colors:
        tup = tuple(c)
        if tup == (0,0,0):
            continue
        mask = ((img_rgb[:,:,0]==c[0]) &
                (img_rgb[:,:,1]==c[1]) &
                (img_rgb[:,:,2]==c[2]) &
                (depth_img <= MAX_DISTANCE))
        if not mask.any():
            continue
        m8 = (mask.astype(np.uint8)*255)
        cnts,_ = cv2.findContours(m8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        pts = np.vstack([cnt.reshape(-1,2) for cnt in cnts])
        x0,y0 = pts.min(axis=0); x1,y1 = pts.max(axis=0)
        unknown_bboxes.append({"color":tup, "contour":pts, "bbox":(x0,y0,x1,y1)})
        # draw raw mask box
        cv2.rectangle(display_unknown, (x0,y0), (x1,y1), BOX_COLOR_UNKNOWN, 1)

    print(f"Raw mask instances within {MAX_DISTANCE}m:", len(unknown_bboxes))

    # 6. get 3D detections
    all_objs = client.simListSceneObjects(name_regex='.*')
    cam_info = client.simGetCameraInfo(CAMERA_NAME, VEHICLE_NAME)
    cam_pos, cam_ori = cam_info.pose.position, cam_info.pose.orientation

    needed_3d = []
    for nm in all_objs:
        label = id_to_label.get(nm, "").lower()
        if not any(k in label for k in KEYWORDS):
            continue
        pose   = client.simGetObjectPose(nm)
        cam_pt = world_to_camera([pose.position], cam_pos, cam_ori)[0]
        if 0 < cam_pt[0] <= MAX_DISTANCE:
            needed_3d.append(nm)

    print(f"3D-detected within {MAX_DISTANCE}m (forward):")
    for n in needed_3d:
        print(" -", id_to_label[n])

    diff = [n for n in needed_3d if n not in seg_detected]
    print("In 3D but not in segmentation:")
    for n in diff:
        print(" -", id_to_label[n])

    # 7. project + match using IoU / centroid / ICP
    centers, pixels = [], []
    for nm in needed_3d:
        pose   = client.simGetObjectPose(nm)
        cam_pt = world_to_camera([pose.position], cam_pos, cam_ori)[0]
        centers.append({"name":nm, "pt":cam_pt})
    pixels = project_to_image([c["pt"] for c in centers], cam_info, W, H)

    iou_m, cen_m, icp_m = [], [], []
    for det, uv in zip(centers, pixels):
        if uv is None:
            continue
        ux, uy = uv
        proj_box = np.array([
            [ux-BOX_HALF, uy-BOX_HALF],
            [ux+BOX_HALF, uy-BOX_HALF],
            [ux+BOX_HALF, uy+BOX_HALF],
            [ux-BOX_HALF, uy+BOX_HALF]
        ])
        # IoU
        best_i, bi = 0, None
        for unk in unknown_bboxes:
            bb = unk["bbox"]
            i = box_iou((proj_box.min(axis=0)[0], proj_box.min(axis=0)[1],
                         proj_box.max(axis=0)[0], proj_box.max(axis=0)[1]), bb)
            if i > best_i:
                best_i, bi = i, unk
        if bi and best_i > IOU_THRESHOLD:
            iou_m.append(det["name"])
            x0,y0,x1,y1 = bi["bbox"]
            cv2.rectangle(display_known, (x0,y0),(x1,y1), BOX_COLOR_UNKNOWN, 2)

        # centroid
        bd, bc = float("inf"), None
        for unk in unknown_bboxes:
            x0,y0,x1,y1 = unk["bbox"]
            cx, cy = (x0+x1)/2, (y0+y1)/2
            d = math.hypot(cx-ux, cy-uy)
            if d < bd:
                bd, bc = d, unk
        if bc and bd <= CENTROID_DIST_THRESHOLD:
            cen_m.append(det["name"])
            x0,y0,x1,y1 = bc["bbox"]
            cv2.rectangle(display_known, (x0,y0),(x1,y1), BOX_COLOR_UNKNOWN, 2)

        # ICP
        best_e, bu, Rb, tb = float("inf"), None, None, None
        for unk in unknown_bboxes:
            R_, t_, e_ = icp_2d(proj_box, unk["contour"])
            if e_ < best_e:
                best_e, bu, Rb, tb = e_, unk, R_, t_
        if bu and best_e <= BOX_HALF:
            icp_m.append(det["name"])
            tr = (Rb @ proj_box.T).T + tb
            x0,y0 = tr.min(axis=0).astype(int)
            x1,y1 = tr.max(axis=0).astype(int)
            cv2.rectangle(display_known,(x0,y0),(x1,y1),BOX_COLOR_UNKNOWN,2)

    print("IoU matches:", iou_m)
    print("Centroid matches:", cen_m)
    print("ICP matches:", icp_m)

    # 8. show windows
    cv2.imshow("Corrected 2D BBoxes", display_known)
    cv2.imshow("Unknown Masks Only",   display_unknown)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
