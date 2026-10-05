#!/usr/bin/env python3
"""Checks a running HERCULES team simulator (editor or packaged build) through both RPC servers.

Lists the robots on the drone (41451) and ground vehicle (41452) servers, fetches every robot's
scene, depth, segmentation, ThermalIR and NightVision images and its LiDAR, IMU and GPS data, takes
off and climbs with the first drone, and drives the first Husky forward. Images are saved to --out.
Exits non-zero if a check fails. Expects drones named Drone* and Huskies named Husky* with a
front_center camera, a LidarSensor1 LiDAR and imu/gps sensors, as in ros2/settings/hero_blocks_team.json.

    python PythonClient/hero/smoke_test_team.py --ip 127.0.0.1 --out smoke_out
    python PythonClient/hero/smoke_test_team.py --no-images   # headless (-nullrhi) simulator
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import hercules_cosysairsim as airsim  # noqa: E402

results = []


def check(name, ok, detail=""):
    results.append((name, bool(ok), detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


def save_images(client, vehicle, out):
    reqs = [
        airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False),
        airsim.ImageRequest("front_center", airsim.ImageType.DepthPlanar, True, False),
        airsim.ImageRequest("front_center", airsim.ImageType.Segmentation, False, False),
        airsim.ImageRequest("front_center", airsim.ImageType.ThermalIR, False, False),
        airsim.ImageRequest("front_center", airsim.ImageType.NightVision, False, False),
    ]
    names = ["scene", "depth", "seg", "thermal", "nvg"]
    resps = client.simGetImages(reqs, vehicle_name=vehicle)
    check(f"{vehicle} simGetImages count", len(resps) == len(reqs), f"{len(resps)}/{len(reqs)}")
    import cv2
    for name, r in zip(names, resps):
        if r.pixels_as_float:
            img = np.array(r.image_data_float, dtype=np.float32)
            ok = r.width > 0 and img.size == r.width * r.height and np.isfinite(img).any()
            if ok:
                d = img.reshape(r.height, r.width)
                vis = np.clip(d / max(1e-3, np.percentile(d[np.isfinite(d)], 95)) * 255, 0, 255).astype(np.uint8)
                cv2.imwrite(os.path.join(out, f"{vehicle}_{name}.png"), vis)
            check(f"{vehicle} {name}", ok, f"{r.width}x{r.height} depth range {np.nanmin(img):.1f}..{np.nanmax(img):.1f}" if ok else "")
        else:
            buf = np.frombuffer(r.image_data_uint8, dtype=np.uint8)
            ok = r.width > 0 and buf.size == r.width * r.height * 3
            if ok:
                img = buf.reshape(r.height, r.width, 3)  # RGB
                cv2.imwrite(os.path.join(out, f"{vehicle}_{name}.png"), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                std = float(img.std())
                ok = std > 1.0  # not a blank frame
            check(f"{vehicle} {name}", ok, f"{r.width}x{r.height} std={buf.std():.1f}" if r.width else "empty")
    return resps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="127.0.0.1")
    ap.add_argument("--out", default="smoke_out")
    ap.add_argument("--no-motion", action="store_true")
    ap.add_argument("--no-images", action="store_true", help="skip cameras (e.g. a -nullrhi run)")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    drone = airsim.MultirotorClient(ip=a.ip, port=41451, timeout_value=60)
    drone.confirmConnection()
    car = airsim.CarClient(ip=a.ip, port=41452, timeout_value=60)
    car.confirmConnection()

    dv = drone.listVehicles()
    cv = car.listVehicles()
    check("drone server listVehicles", any(v.startswith("Drone") for v in dv), str(dv))
    check("car server listVehicles", any(v.startswith("Husky") for v in cv), str(cv))
    drones = sorted(v for v in dv if v.startswith("Drone"))
    huskies = sorted(v for v in cv if v.startswith("Husky"))

    if not a.no_images:
        for v in drones:
            save_images(drone, v, a.out)
        for v in huskies:
            save_images(car, v, a.out)

    for client, v in [(drone, d) for d in drones] + [(car, h) for h in huskies]:
        lid = client.getLidarData("LidarSensor1", v)
        n = len(lid.point_cloud) // 3
        check(f"{v} lidar", n > 100, f"{n} points")
        imu = client.getImuData("imu", v)
        g = np.linalg.norm([imu.linear_acceleration.x_val, imu.linear_acceleration.y_val, imu.linear_acceleration.z_val])
        check(f"{v} imu", 5.0 < g < 15.0, f"|a|={g:.2f}")
        gps = client.getGpsData("gps", v)
        check(f"{v} gps", gps.gnss.geo_point.latitude != 0.0, f"lat={gps.gnss.geo_point.latitude:.6f}")

    if not a.no_motion and drones:
        d = drones[0]
        drone.enableApiControl(True, d)
        drone.armDisarm(True, d)
        z0 = drone.simGetVehiclePose(d).position.z_val
        drone.takeoffAsync(vehicle_name=d).join()
        drone.moveToZAsync(z0 - 5.0, 2.0, vehicle_name=d).join()
        time.sleep(1.0)
        z1 = drone.simGetVehiclePose(d).position.z_val
        check(f"{d} takeoff+climb", z0 - z1 > 3.0, f"z {z0:.2f} -> {z1:.2f}")
        if not a.no_images:
            save_images(drone, d, os.path.join(a.out))
        drone.landAsync(vehicle_name=d).join()

    if not a.no_motion and huskies:
        h = huskies[0]
        car.enableApiControl(True, h)
        p0 = car.simGetVehiclePose(h).position
        ctl = airsim.CarControls()
        ctl.throttle = 0.6
        car.setCarControls(ctl, h)
        time.sleep(4.0)
        ctl.throttle = 0.0
        ctl.brake = 1.0
        car.setCarControls(ctl, h)
        time.sleep(1.0)
        p1 = car.simGetVehiclePose(h).position
        dist = float(np.hypot(p1.x_val - p0.x_val, p1.y_val - p0.y_val))
        check(f"{h} drive", dist > 0.5, f"moved {dist:.2f} m")
        car.enableApiControl(False, h)

    failed = [r for r in results if not r[1]]
    print(f"\n{len(results) - len(failed)}/{len(results)} checks passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
