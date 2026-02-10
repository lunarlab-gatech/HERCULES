#!/usr/bin/env python3
import setup_path                    # ensure hercules is on PYTHONPATH
import hercules as airsim
import cv2
import numpy as np
import open3d as o3d

# Camera and detection settings
CAMERA_NAME = "front_center"
IMAGE_TYPE  = airsim.ImageType.Scene

# Transformation from AirSim camera FRD frame (Forward, Right, Down)
# to Open3D RUF frame (Right, Up, Forward)
def _cam_to_o3d(pt):
    x_fwd, y_right, z_down = pt
    return [y_right, -z_down, x_fwd]


def make_line_set(det):
    """Given a single detection, return an Open3D LineSet of its 3D box in the Open3D frame."""
    # 1) Extract raw min/max corners from AirSim box3D
    pmin, pmax = det.box3D.min, det.box3D.max
    xs = sorted([pmin.x_val, pmax.x_val])
    ys = sorted([pmin.y_val, pmax.y_val])
    zs = sorted([pmin.z_val, pmax.z_val])

    # 2) Build the 8 corners in FRD order then map to RUF
    corners_cam = [
        (xs[i], ys[j], zs[k])
        for i in (0, 1)
        for j in (0, 1)
        for k in (0, 1)
    ]
    corners_o3d = np.array([_cam_to_o3d(c) for c in corners_cam], dtype=float)

    # 3) Define box edges (12 segments)
    edges = [
        [0,1],[1,3],[3,2],[2,0],  # bottom face
        [4,5],[5,7],[7,6],[6,4],  # top face
        [0,4],[1,5],[2,6],[3,7]   # vertical edges
    ]

    # 4) Create LineSet
    ls = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(corners_o3d),
        lines=o3d.utility.Vector2iVector(edges)
    )
    ls.colors = o3d.utility.Vector3dVector([[0, 1, 0]] * len(edges))
    return ls


def main():
    # Connect to AirSim
    client = airsim.CarClient(port=41452)
    client.confirmConnection()

    # Set up mesh filter (keep Cylinder* filter)
    client.simClearDetectionMeshNames(CAMERA_NAME, IMAGE_TYPE)
    client.simAddDetectionFilterMeshName(CAMERA_NAME, IMAGE_TYPE, "Cylinder*")
    client.simSetDetectionFilterRadius(CAMERA_NAME, IMAGE_TYPE, 200 * 100)

    # OpenCV window for 2D detections
    cv2.namedWindow("2D Detections", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)

    # Open3D window for 3D detections
    vis = o3d.visualization.Visualizer()
    vis.create_window("3D Detections", width=800, height=600)
    ctr = vis.get_view_control()
    ctr.set_constant_z_near(0.01)
    ctr.set_constant_z_far(1e6)

    # Add camera frame axes at origin
    cam_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    vis.add_geometry(cam_frame)

    current_ls = []
    try:
        while True:
            # 1) Grab image
            resp = client.simGetImages([
                airsim.ImageRequest(CAMERA_NAME, IMAGE_TYPE, False, True)
            ])[0]
            if not resp.image_data_uint8:
                continue
            img = cv2.imdecode(
                np.frombuffer(resp.image_data_uint8, np.uint8),
                cv2.IMREAD_COLOR
            )

            # 2) Fetch detections
            dets = client.simGetDetections(CAMERA_NAME, IMAGE_TYPE) or []

            # 3) Draw 2D boxes
            for d in dets:
                x0, y0 = int(d.box2D.min.x_val), int(d.box2D.min.y_val)
                x1, y1 = int(d.box2D.max.x_val), int(d.box2D.max.y_val)
                cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 0), 2)
                cv2.putText(img, d.name, (x0, max(0, y0 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow("2D Detections", img)

            # 4) Update 3D boxes
            for ls in current_ls:
                vis.remove_geometry(ls, reset_bounding_box=False)
            current_ls.clear()
            for d in dets:
                ls = make_line_set(d)
                vis.add_geometry(ls, reset_bounding_box=False)
                current_ls.append(ls)

            # 5) Poll events and render
            vis.poll_events()
            vis.update_renderer()

            # 6) Exit on 'q'
            if cv2.waitKey(50) & 0xFF == ord('q'):
                break

    finally:
        cv2.destroyAllWindows()
        vis.destroy_window()


if __name__ == "__main__":
    main()
