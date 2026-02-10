#!/usr/bin/env python3
"""
hercules_data_collector_single_agent.py
NOTE!!! This script is for a single agent type only -- either UGV or UAV running in sim, 
not both simultaenously.

Pauses the HERCULES sim, collects perfectly synchronized sensor data
at specified rates (IMU & odom at 200 Hz, cameras at 20 Hz, LiDAR at 10 Hz),
and saves:

  • imu.txt      — one line per timestamped IMU sample  
  • odom.txt     — one line per timestamped odometry sample  
  • rgb/, depth/, seg/  — per-frame images  
  • lidar/       — per-frame point clouds (.npy)

All timestamps are in pure sim-seconds (t = step*dt), to six decimals.
"""

import setup_path
import os
import argparse
import numpy as np
import cv2
import hercules as airsim

def save_image(resp, out_dir, t, ext):
    """Save AirSim image with relative timestamp."""
    path = os.path.join(out_dir, f"{t:.6f}{ext}")
    if resp.pixels_as_float:
        data = np.array(resp.image_data_float, dtype=np.float32)
        img = data.reshape(resp.height, resp.width)
        img = (255 * (img - img.min()) / (img.max() - img.min())).astype(np.uint8)
        cv2.imwrite(path, img)
    else:
        arr = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)
        img = arr.reshape(resp.height, resp.width, 3)
        cv2.imwrite(path, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

def main():
    parser = argparse.ArgumentParser("Collect synchronized AirSim data")
    parser.add_argument("--duration",   type=float, default=60.0,
                        help="Total simulation time in seconds")
    parser.add_argument("--outdir",     type=str,   default="dataset",
                        help="Root output directory")
    parser.add_argument("--vehicle",    type=str,   default="Husky1",
                        help="Vehicle name in settings.json")
    parser.add_argument("--camera",     type=str,   default="front_center",
                        help="Camera name in settings.json")
    parser.add_argument("--port",       type=int,   default=41452,
                        help="API server port for this vehicle")
    parser.add_argument("--client-type", choices=["car","multirotor"], default="car",
                        help="Which AirSim client to use (CarClient vs MultirotorClient)")
    args = parser.parse_args()

    # instantiate the correct AirSim client
    if args.client_type == "car":
        client = airsim.CarClient(port=args.port)
    else:
        client = airsim.MultirotorClient(port=args.port)

    client.confirmConnection()
    client.enableApiControl(True, vehicle_name=args.vehicle)
    client.simPause(True)

    # desired rates
    imu_hz, odom_hz = 200.0, 20.0
    cam_hz, lidar_hz = 20.0, 10.0
    dt = 1.0 / imu_hz

    # compute subsampling steps
    odom_step  = int(round(imu_hz / odom_hz))
    cam_step   = int(round(imu_hz / cam_hz))
    lidar_step = int(round(imu_hz / lidar_hz))
    total_steps = int(round(args.duration / dt))

    # prepare output directories and files
    os.makedirs(args.outdir, exist_ok=True)
    imu_f  = open(os.path.join(args.outdir, "imu.txt"), 'w')
    odom_f = open(os.path.join(args.outdir, "odom.txt"), 'w')
    for sub in ("rgb","depth","seg","lidar"):
        os.makedirs(os.path.join(args.outdir, sub), exist_ok=True)

    print(f"Running {total_steps} steps at {imu_hz:.0f} Hz (dt={dt:.4f}s)…")

    for step in range(1, total_steps+1):
        # 1) advance sim by dt (then it auto-pauses)
        client.simContinueForTime(dt)

        # 2) compute exact sim-time
        t = step * dt

        # 3) IMU @ 200 Hz
        imu = client.getImuData(vehicle_name=args.vehicle)
        la, av = imu.linear_acceleration, imu.angular_velocity
        imu_f.write(f"{t:.6f} "
                    f"{la.x_val:.6f} {la.y_val:.6f} {la.z_val:.6f} "
                    f"{av.x_val:.6f} {av.y_val:.6f} {av.z_val:.6f}\n")

        # 4) Odometry @ 20 Hz
        if step % odom_step == 0:
            # choose the right state call
            if args.client_type == "car":
                st = client.getCarState(vehicle_name=args.vehicle)
            else:
                st = client.getMultirotorState(vehicle_name=args.vehicle)
            pos = st.kinematics_estimated.position
            ori = st.kinematics_estimated.orientation
            odom_f.write(f"{t:.6f} "
                         f"{pos.x_val:.6f} {pos.y_val:.6f} {pos.z_val:.6f} "
                         f"{ori.w_val:.6f} {ori.x_val:.6f} {ori.y_val:.6f} {ori.z_val:.6f}\n")

        # 5) Cameras @ 20 Hz
        if step % cam_step == 0:
            reqs = [
                airsim.ImageRequest(args.camera, airsim.ImageType.Scene,       False, False),
                airsim.ImageRequest(args.camera, airsim.ImageType.DepthPlanar, True,  False),
                airsim.ImageRequest(args.camera, airsim.ImageType.Segmentation,False, False),
            ]
            imgs = client.simGetImages(reqs, vehicle_name=args.vehicle)
            save_image(imgs[0], os.path.join(args.outdir, "rgb"),   t, ".png")
            save_image(imgs[1], os.path.join(args.outdir, "depth"), t, ".png")
            save_image(imgs[2], os.path.join(args.outdir, "seg"),   t, ".png")

        # 6) LiDAR @ 10 Hz
        if step % lidar_step == 0:
            ld = client.getLidarData(lidar_name="LidarSensor1",
                                     vehicle_name=args.vehicle)
            pts = np.array(ld.point_cloud, dtype=np.float32).reshape(-1, 3)
            np.save(os.path.join(args.outdir, "lidar", f"{t:.6f}.npy"), pts)

    # close files and resume real-time sim
    imu_f.close()
    odom_f.close()
    client.simPause(False)
    print("Done. Data saved under:", args.outdir)


if __name__ == "__main__":
    main()
