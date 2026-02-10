#!/usr/bin/env python3
"""
Simple script to retrieve a scene image and project a 3D bounding box onto it using AirSim's built-in projection matrix.
"""
import setup_path
import hercules as airsim
import numpy as np
from PIL import Image, ImageDraw, ImageFont

def to_numpy_vector(v):
    return np.array([v.x_val, v.y_val, v.z_val])

def quat_to_rot_matrix(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
        [2*(x*y + z*w),   1-2*(x*x+z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x*x+y*y)]
    ])

def main():
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("Connected to AirSim multirotor client.")

    # 1. get scene
    resp = client.simGetImages([airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)], vehicle_name="Drone1")[0]
    img1d = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)
    w_img, h_img = resp.width, resp.height
    scene = Image.fromarray(img1d.reshape(h_img, w_img, 3))
    scene.save("scene.png")
    print("Saved scene.png")

    # 2. camera pose
    cam_info = client.simGetCameraInfo("front_center", vehicle_name="Drone1")
    cam_p = to_numpy_vector(cam_info.pose.position)
    cam_q = np.array([cam_info.pose.orientation.w_val,
                      cam_info.pose.orientation.x_val,
                      cam_info.pose.orientation.y_val,
                      cam_info.pose.orientation.z_val])

    # 3. get projection matrix P (4×4 row-major)
    P = np.array(cam_info.proj_mat.matrix, dtype=np.float64).reshape((4,4))
    print("P =")
    print(P)

    # 4. object pose
    obj_pose = client.simGetObjectPose("StaticMeshActor_1")
    # obj_pose = client.simGetObjectPose("BP_SplineHuman_Type10_C_UAID_E08F4CF5208A437A02_1596611129")
    obj_p = to_numpy_vector(obj_pose.position)
    obj_q = np.array([obj_pose.orientation.w_val,
                      obj_pose.orientation.x_val,
                      obj_pose.orientation.y_val,
                      obj_pose.orientation.z_val])

    # 5. compute 8 bounding-box corners in world
    dx, dy, dz = 1.0, 1.0, 1.0
    half = np.array([dx/2, dy/2, dz/2])
    # user-specified NED order
    local = np.array([
        [-half[0], +half[1], +half[2]],
        [-half[0], +half[1], -half[2]],
        [-half[0], -half[1], +half[2]],
        [-half[0], -half[1], -half[2]],
        [+half[0], +half[1], +half[2]],
        [+half[0], +half[1], -half[2]],
        [+half[0], -half[1], +half[2]],
        [+half[0], -half[1], -half[2]],
    ])
    R_obj = quat_to_rot_matrix(obj_q)
    world_corners = (R_obj @ local.T).T + obj_p

    # 6. transform corners to camera frame
    R_cam = quat_to_rot_matrix(cam_q)
    cam_corners = (R_cam.T @ (world_corners - cam_p).T).T
    print("Corners in camera frame:")
    for i,c in enumerate(cam_corners): print(f"  {i}: {c}")

    # 7. project via P
    pts_h = np.hstack([cam_corners, np.ones((8,1))])      # Nx4
    clip = (P @ pts_h.T).T                                # Nx4
    ndc  = clip[:,:3] / clip[:,3:4]                       # normalize
    # NDC x,y in [-1,1]; map to pixel
    # u = ( ndc[:,0]*0.5 + 0.5 ) * w_img
    # v = ( 1 - (ndc[:,1]*0.5 + 0.5) ) * h_img

    # correct:flip X the same way you dropped the flip on Y:
    u = (1 - (ndc[:,0]*0.5 + 0.5)) * w_img
    # correct: map NDC Y directly into pixel Y
    v = (ndc[:,1]*0.5 + 0.5) * h_img

    proj2d = np.stack([u,v], axis=1)

    # 8. draw
    img = scene.copy(); draw = ImageDraw.Draw(img)
    lines = [(0,1),(0,2),(0,4),(1,3),(1,5),(2,3),(2,6),(3,7),(4,5),(4,6),(5,7),(6,7)]
    font = ImageFont.load_default()
    for i,j in lines:
        p1 = tuple(proj2d[i].astype(int)); p2 = tuple(proj2d[j].astype(int))
        draw.line([p1,p2], fill="red", width=2)
    for i,(x,y) in enumerate(proj2d):
        draw.text((int(x)+5,int(y)+5), str(i), fill="red", font=font)
    img.save("scene_bbox.png")
    print("Saved scene_bbox.png with projected box.")

if __name__ == '__main__': main()