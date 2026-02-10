#!/usr/bin/env python3.10
"""
hercules_multi_vehicle_data_collector.py

Pauses the sim on both clients, then in each loop unpauses
and steps each vehicle by DT, before pausing again and gathering data.
"""

import setup_path
import os
import numpy as np
import cv2
import hercules as airsim

# Configuration
DURATION   = 660.0        # seconds
DT_RATE    = 200.0        # Hz
DT         = 1.0 / DT_RATE
OUTDIR     = "/media/sgarimella34/hercules-collect/raw_data_hercules/test1_2uav2ugv"

DRONE_NAMES = ["Drone1", "Drone2"]
HUSKY_NAMES = ["Husky1", "Husky2"]

CAMERA_NAME = "front_center"
LIDAR_NAME  = "LidarSensor1"

DRONE_PORT = 41451
HUSKY_PORT = 41452

# instantiate clients
drone_client = airsim.MultirotorClient(port=DRONE_PORT)
husky_client = airsim.CarClient      (port=HUSKY_PORT)
drone_client.confirmConnection()
husky_client.confirmConnection()

# enable API control
for n in DRONE_NAMES: drone_client.enableApiControl(True, vehicle_name=n)
for n in HUSKY_NAMES: husky_client.enableApiControl(True, vehicle_name=n)

# globally pause both sims
drone_client.simPause(True)
husky_client.simPause(True)

# set up output dirs and file handles
os.makedirs(OUTDIR, exist_ok=True)
files = {}
for name in DRONE_NAMES + HUSKY_NAMES:
    base = os.path.join(OUTDIR, name)
    os.makedirs(base, exist_ok=True)
    handles = {
        'imu':  open(os.path.join(base, 'imu.txt'),  'w'),
        'odom': open(os.path.join(base, 'odom.txt'), 'w'),
    }
    for fld in ('rgb','depth','seg','lidar'):
        p = os.path.join(base, fld)
        os.makedirs(p, exist_ok=True)
        handles[fld] = p
    files[name] = handles

odom_step  = int(round(DT_RATE/20.0))   # 20 Hz
cam_step   = odom_step
lidar_step = int(round(DT_RATE/10.0))  # 10 Hz

total_steps = int(round(DURATION/DT))
print(f"Collecting {total_steps} steps @ {DT_RATE} Hz…")

for step in range(1, total_steps+1):
    t = step * DT

    # --- unpause both ---
    drone_client.simPause(False)
    husky_client.simPause(False)

    # --- step both by DT ---
    drone_client.simContinueForTime(DT)
    husky_client.simContinueForTime(DT)

    # --- pause both ---
    drone_client.simPause(True)
    husky_client.simPause(True)

    # --- collect drone data ---
    for name in DRONE_NAMES:
        # IMU
        imu = drone_client.getImuData(vehicle_name=name)
        la, av = imu.linear_acceleration, imu.angular_velocity
        files[name]['imu'].write(
            f"{t:.6f} {la.x_val:.6f} {la.y_val:.6f} {la.z_val:.6f} "
            f"{av.x_val:.6f} {av.y_val:.6f} {av.z_val:.6f}\n"
        )
        # Odom @20 Hz
        if step % odom_step == 0:
            st = drone_client.getMultirotorState(vehicle_name=name)
            p = st.kinematics_estimated.position
            o = st.kinematics_estimated.orientation
            files[name]['odom'].write(
                f"{t:.6f} {p.x_val:.6f} {p.y_val:.6f} {p.z_val:.6f} "
                f"{o.w_val:.6f} {o.x_val:.6f} {o.y_val:.6f} {o.z_val:.6f}\n"
            )
        # Cameras @20 Hz
        if step % cam_step == 0:
            reqs = [
                airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.Scene, False, False),
                airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.DepthPlanar, True, False),
                airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.Segmentation, False, False),
            ]
            imgs = drone_client.simGetImages(reqs, vehicle_name=name)
            for img, key in zip(imgs, ('rgb','depth','seg')):
                if img.pixels_as_float:
                    arr = np.array(img.image_data_float, dtype=np.float32)
                    arr = arr.reshape(img.height, img.width)
                    norm = ((arr-arr.min())/(arr.max()-arr.min())*255).astype(np.uint8)
                    cv2.imwrite(os.path.join(files[name][key], f"{t:.6f}.png"), norm)
                else:
                    arr = np.frombuffer(img.image_data_uint8, dtype=np.uint8)
                    arr = arr.reshape(img.height, img.width, 3)
                    cv2.imwrite(
                        os.path.join(files[name][key], f"{t:.6f}.png"),
                        cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                    )
        # LiDAR @10 Hz
        if step % lidar_step == 0:
            ld = drone_client.getLidarData(lidar_name=LIDAR_NAME, vehicle_name=name)
            pts = np.array(ld.point_cloud, dtype=np.float32).reshape(-1,3)
            np.save(os.path.join(files[name]['lidar'], f"{t:.6f}.npy"), pts)

    # --- collect husky data ---
    for name in HUSKY_NAMES:
        imu = husky_client.getImuData(vehicle_name=name)
        la, av = imu.linear_acceleration, imu.angular_velocity
        files[name]['imu'].write(
            f"{t:.6f} {la.x_val:.6f} {la.y_val:.6f} {la.z_val:.6f} "
            f"{av.x_val:.6f} {av.y_val:.6f} {av.z_val:.6f}\n"
        )
        if step % odom_step == 0:
            st = husky_client.getCarState(vehicle_name=name)
            p  = st.kinematics_estimated.position
            o  = st.kinematics_estimated.orientation
            files[name]['odom'].write(
                f"{t:.6f} {p.x_val:.6f} {p.y_val:.6f} {p.z_val:.6f} "
                f"{o.w_val:.6f} {o.x_val:.6f} {o.y_val:.6f} {o.z_val:.6f}\n"
            )
        if step % cam_step == 0:
            reqs = [
                airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.Scene, False, False),
                airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.DepthPlanar, True, False),
                airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.Segmentation, False, False),
            ]
            imgs = husky_client.simGetImages(reqs, vehicle_name=name)
            for img, key in zip(imgs, ('rgb','depth','seg')):
                if img.pixels_as_float:
                    arr = np.array(img.image_data_float, dtype=np.float32)
                    arr = arr.reshape(img.height, img.width)
                    norm = ((arr-arr.min())/(arr.max()-arr.min())*255).astype(np.uint8)
                    cv2.imwrite(os.path.join(files[name][key], f"{t:.6f}.png"), norm)
                else:
                    arr = np.frombuffer(img.image_data_uint8, dtype=np.uint8)
                    arr = arr.reshape(img.height, img.width, 3)
                    cv2.imwrite(
                        os.path.join(files[name][key], f"{t:.6f}.png"),
                        cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                    )
        if step % lidar_step == 0:
            ld  = husky_client.getLidarData(lidar_name=LIDAR_NAME, vehicle_name=name)
            pts = np.array(ld.point_cloud, dtype=np.float32).reshape(-1,3)
            np.save(os.path.join(files[name]['lidar'], f"{t:.6f}.npy"), pts)

# cleanup
drone_client.simPause(False)
husky_client.simPause(False)
for h in files.values():
    h['imu'].close()
    h['odom'].close()

print("Done. Data saved under:", OUTDIR)
