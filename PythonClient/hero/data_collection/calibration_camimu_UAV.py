#!/usr/bin/env python3

import setup_path
import hercules as airsim
import numpy as np
import time
import math

# List your multirotor names as in settings.json
DRONE_NAMES = [
    "Drone1",
    "Drone2"
]

def parallel_join(futures):
    """Helper to wait for a list of AirSim futures."""
    for f in futures:
        f.join()

def euler_from_quaternion(q):
    """Extract yaw (rad) from AirSim Quaternionr (w, x, y, z)."""
    w, x, y, z = q.w_val, q.x_val, q.y_val, q.z_val
    return math.atan2(
        2.0*(w*z + x*y),
        1.0 - 2.0*(y*y + z*z)
    )

def rotated_offset(dx, dy, yaw):
    """Rotate a local (dx, dy) vector into world frame using yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c*dx - s*dy, s*dx + c*dy

def run_calibration_on_all(client, drone_names):
    # 0) Record each drone’s starting (x, y, z, yaw)
    starts, target_z = {}, {}
    for name in drone_names:
        p = client.simGetVehiclePose(vehicle_name=name)
        yaw0 = euler_from_quaternion(p.orientation)
        starts[name]   = (p.position.x_val, p.position.y_val, yaw0)
        target_z[name] = p.position.z_val - 2.0

    # 1) Takeoff and climb to (start_z - 2.0)
    for name in drone_names:
        client.takeoffAsync(vehicle_name=name).join()
        client.moveToZAsync(target_z[name], 5,
                             vehicle_name=name).join()

    # time.sleep(1)

    # 2) Move 0.5 m forward (body-frame) holding start yaw
    futures = []
    for name in drone_names:
        x0, y0, yaw0 = starts[name]
        dx, dy       = rotated_offset(0.5, 0.0, yaw0)
        futures.append(
            client.moveToPositionAsync(
                x0 + dx, y0 + dy, target_z[name], 2,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=yaw0),
                vehicle_name=name
            )
        )
    parallel_join(futures)
    time.sleep(2)

    # 3) Attitude oscillations + yaw‐rate (hold altitude)
    seq = [
        ( np.pi/16,  0,          0, 1 ),
        (-np.pi/16,  0,          0, 1 ),
        ( 0,        -np.pi/20,   0, 1 ),
        ( 0,         np.pi/20,   0, 1 ),
        ( 0,         0,    -np.pi/10, 1 ),
        ( 0,         0,     np.pi/10, 2 ),
        ( 0,         0,    -np.pi/20, 1 ),
    ]
    for roll, pitch, yaw_rate, dur in seq:
        # perform roll/pitch/yaw‐rate at constant Z
        futs = [
            client.moveByRollPitchYawrateZAsync(
                roll, pitch, yaw_rate,
                target_z[name], dur,
                vehicle_name=name
            )
            for name in drone_names
        ]
        parallel_join(futs)

        # brief hover (zero yaw‐rate) to settle
        zeros = [
            client.moveByVelocityBodyFrameAsync(
                0, 0, 0, 1,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(),  # zero yaw rate :contentReference[oaicite:0]{index=0}
                vehicle_name=name
            )
            for name in drone_names
        ]
        parallel_join(zeros)

    print("CHECKPOINT1")

    # 4) Body‐frame velocity bursts
    vel_seq = [
        ( 0,  0.5,  0, 1),
        ( 0, -0.5,  0, 2),
        ( 0,  0.5, -0.2, 2),
        ( 0, -0.5,  0.2, 2),
        (-0.5, 0,   0, 1),
        ( 0.5, 0,   0, 2),
        (-0.5, 0,   0, 1),
        ( 0,  0.3, 0, 1),
        ( 0,  0,  -1, 0.5),
        ( 0,  0,   1, 1),
    ]
    for vx, vy, vz, dur in vel_seq:
        futs = [
            client.moveByVelocityBodyFrameAsync(
                vx, vy, vz, dur,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(),  # zero yaw rate :contentReference[oaicite:1]{index=1}
                vehicle_name=name
            )
            for name in drone_names
        ]
        parallel_join(futs)

        # hover to settle between bursts
        stops = [
            client.moveByVelocityBodyFrameAsync(
                0, 0, 0, 1,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(),  # zero yaw rate
                vehicle_name=name
            )
            for name in drone_names
        ]
        parallel_join(stops)

    print("CHECKPOINT2")
    print("Calibration motion finished on all drones.")

    # 5) Hover in place; leave API control for next script
    for name in drone_names:
        client.hoverAsync(vehicle_name=name)

if __name__ == "__main__":
    client = airsim.MultirotorClient(port=41451)
    client.confirmConnection()

    # enable API control and arm every drone
    for name in DRONE_NAMES:
        client.enableApiControl(True, vehicle_name=name)
        client.armDisarm(True, vehicle_name=name)

    time.sleep(1.0)
    run_calibration_on_all(client, DRONE_NAMES)
