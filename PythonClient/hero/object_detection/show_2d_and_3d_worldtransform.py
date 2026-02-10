#!/usr/bin/env python3
import setup_path                    # ensure hercules is on PYTHONPATH
import hercules as airsim
import cv2, numpy as np, open3d as o3d
import time, math

CAMERA_NAME = "front_center"
IMAGE_TYPE  = airsim.ImageType.Scene

# Primary detection mesh patterns (wildcards)
DETECTION_MESH_PATTERNS = ["Sportscar*", "BP_SplineHuman_Type10.*", "SK_Survival_Character*"]  # adjust if needed

# Fallback actor patterns (e.g., blueprint instances whose nested meshes aren't surfaced by detection)
FALLBACK_ACTOR_PATTERNS = ["BP_SplineHuman_Type10.*"]  # regex-style to match runtime instance names

def quaternion_to_rot_matrix(q):
    w, x, y, z = q.w_val, q.x_val, q.y_val, q.z_val
    n = np.linalg.norm([w, x, y, z])
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ])

def world_T_cam_from_info(cam_info):
    p = cam_info.pose.position
    R = quaternion_to_rot_matrix(cam_info.pose.orientation)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [p.x_val, p.y_val, p.z_val]
    return T  # maps camera-frame → world-frame

def _cam_to_o3d(pt):
    # AirSim camera FRD → Open3D RUF
    x_fwd, y_right, z_down = pt
    return [y_right, -z_down, x_fwd]

def detection_matches_actor(det_name: str, actor_name: str) -> bool:
    # loose heuristic so similar naming doesn’t get double-dropped
    return (
        det_name.startswith(actor_name)
        or actor_name.startswith(det_name)
        or det_name in actor_name
        or actor_name in det_name
    )

def discover_fallback_actors(client, patterns):
    matched = []
    for pat in patterns:
        try:
            found = client.simListSceneObjects(pat) or []
        except Exception as e:
            print(f"[fallback discovery] error listing with pattern '{pat}': {e}")
            found = []
        if found:
            print(f"[fallback discovery] pattern '{pat}' matched: {found}")
        else:
            print(f"[fallback discovery] pattern '{pat}' matched nothing.")
        for o in found:
            if o not in matched:
                matched.append(o)
    return matched

def refresh_fallback_and_segmentation(client, patterns, known_fallbacks, next_seg_id):
    found = discover_fallback_actors(client, patterns)
    for obj in found:
        if obj not in known_fallbacks:
            success = client.simSetSegmentationObjectID(obj, next_seg_id, False)
            print(f"[segmentation] set ID {next_seg_id} for '{obj}': {success}")
            known_fallbacks.add(obj)
            next_seg_id += 1
            if next_seg_id > 250:
                print("Warning: segmentation ID approaching upper bound; further IDs may collide or be invalid.")
    return next_seg_id

def make_box_lineset_from_detection(d):
    pmin, pmax = d.box3D.min, d.box3D.max
    xs = sorted([pmin.x_val, pmax.x_val])
    ys = sorted([pmin.y_val, pmax.y_val])
    zs = sorted([pmin.z_val, pmax.z_val])
    corners_cam = np.array([[xs[i], ys[j], zs[k]]
                             for i in (0,1) for j in (0,1) for k in (0,1)])
    corners_o3d = np.array([_cam_to_o3d(c) for c in corners_cam])
    edges = [
        [0,1],[1,3],[3,2],[2,0],
        [4,5],[5,7],[7,6],[6,4],
        [0,4],[1,5],[2,6],[3,7]
    ]
    ls = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(corners_o3d),
        lines=o3d.utility.Vector2iVector(edges)
    )
    ls.colors = o3d.utility.Vector3dVector([[0,1,0]] * len(edges))
    return ls

def main():
    client = airsim.CarClient(port=41452)
    client.confirmConnection()

    # detection radius (cm)
    client.simSetDetectionFilterRadius(CAMERA_NAME, IMAGE_TYPE, 200 * 100)

    # set wildcard detection filters once
    client.simClearDetectionMeshNames(CAMERA_NAME, IMAGE_TYPE)
    for pat in DETECTION_MESH_PATTERNS:
        client.simAddDetectionFilterMeshName(CAMERA_NAME, IMAGE_TYPE, pat)

    # fallback / blueprint actor state
    known_fallbacks = set()
    next_seg_id = 200
    last_refresh = 0.0
    REFRESH_INTERVAL = 2.0  # seconds

    # initial discovery & segmentation assignment
    next_seg_id = refresh_fallback_and_segmentation(
        client, FALLBACK_ACTOR_PATTERNS, known_fallbacks, next_seg_id
    )
    last_refresh = time.time()

    # visualization
    cv2.namedWindow("2D Detections", cv2.WINDOW_NORMAL)
    vis = o3d.visualization.Visualizer()
    vis.create_window("3D Detections")
    ctr = vis.get_view_control()
    ctr.set_constant_z_near(0.01); ctr.set_constant_z_far(1e6)
    vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0))  # camera origin

    geoms = []

    try:
        while True:
            now = time.time()
            if now - last_refresh > REFRESH_INTERVAL:
                next_seg_id = refresh_fallback_and_segmentation(
                    client, FALLBACK_ACTOR_PATTERNS, known_fallbacks, next_seg_id
                )
                last_refresh = now

            # --- image grab ---
            img_r = client.simGetImages([
                airsim.ImageRequest(CAMERA_NAME, IMAGE_TYPE, False, True)
            ])[0]
            if not img_r.image_data_uint8:
                continue
            img = cv2.imdecode(np.frombuffer(img_r.image_data_uint8, np.uint8),
                               cv2.IMREAD_COLOR)

            # get image size robustly (AirSim image response has width/height; fallback to decoded image)
            if hasattr(img_r, "height") and hasattr(img_r, "width"):
                h, w = img_r.height, img_r.width  # documented usage of response.width/height for image APIs. :contentReference[oaicite:3]{index=3}
            else:
                h, w = img.shape[:2]

            # --- camera info and transforms ---
            cam_info = client.simGetCameraInfo(CAMERA_NAME)
            world_T_cam = world_T_cam_from_info(cam_info)
            cam_T_world = np.linalg.inv(world_T_cam)

            # --- detections ---
            dets_raw = client.simGetDetections(CAMERA_NAME, IMAGE_TYPE) or []
            print(f"[DEBUG] raw detection names: {[d.name for d in dets_raw]}")

            # clear previous dynamic 3D geometry (keep camera frame)
            for g in geoms:
                vis.remove_geometry(g, reset_bounding_box=False)
            geoms.clear()

            # draw any real detections (2D + 3D)
            for d in dets_raw:
                # 2D
                x0 = int(d.box2D.min.x_val); y0 = int(d.box2D.min.y_val)
                x1 = int(d.box2D.max.x_val); y1 = int(d.box2D.max.y_val)
                cv2.rectangle(img, (x0, y0), (x1, y1), (0,255,0), 2)
                cv2.putText(img, d.name, (x0, max(0, y0 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

                # 3D box
                ls = make_box_lineset_from_detection(d)
                vis.add_geometry(ls, reset_bounding_box=False)
                geoms.append(ls)

                # relative pose triad
                rp = d.relative_pose.position
                cam_c = np.array([rp.x_val, rp.y_val, rp.z_val])
                o3d_c = np.array(_cam_to_o3d(cam_c))
                frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
                frame.translate(o3d_c)
                vis.add_geometry(frame, reset_bounding_box=False)
                geoms.append(frame)

            # --- fallback for blueprint actors not appearing in detections ---
            for obj in sorted(known_fallbacks):
                seen = any(detection_matches_actor(d.name, obj) for d in dets_raw)
                if seen:
                    continue

                # correct single-argument call to get object pose. :contentReference[oaicite:4]{index=4}
                pose = client.simGetObjectPose(obj)
                if pose is None:
                    continue
                pos = pose.position
                if not (math.isfinite(pos.x_val) and math.isfinite(pos.y_val) and math.isfinite(pos.z_val)):
                    continue  # invalid pose

                world_pos = np.array([pos.x_val, pos.y_val, pos.z_val, 1.0])
                cam_coord = cam_T_world @ world_pos  # in camera FRD

                x_fwd, y_right, z_down = cam_coord[:3]
                if x_fwd <= 0.0:
                    continue  # behind the camera

                # draw fallback 3D triad
                o3d_pos = np.array(_cam_to_o3d([x_fwd, y_right, z_down]))
                frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
                frame.translate(o3d_pos)
                vis.add_geometry(frame, reset_bounding_box=False)
                geoms.append(frame)

                # project into 2D and annotate
                fov_rad = np.deg2rad(cam_info.fov)  # CameraInfo.fov is in degrees. :contentReference[oaicite:5]{index=5}
                fy = h / (2 * math.tan(fov_rad / 2))
                fx = fy  # assume square pixels
                cx, cy = w / 2.0, h / 2.0
                u = int((y_right / x_fwd) * fx + cx)
                v = int((z_down / x_fwd) * fy + cy)
                if 0 <= u < w and 0 <= v < h:
                    cv2.drawMarker(img, (u, v), (0, 0, 255),
                                   markerType=cv2.MARKER_CROSS,
                                   markerSize=10, thickness=2)
                    cv2.putText(img, obj, (u + 5, v + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

            # --- display ---
            cv2.imshow("2D Detections", img)
            vis.poll_events()
            vis.update_renderer()

            if cv2.waitKey(50) & 0xFF == ord('q'):
                break

    finally:
        cv2.destroyAllWindows()
        vis.destroy_window()

if __name__ == "__main__":
    main()
