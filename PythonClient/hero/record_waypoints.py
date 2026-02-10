#!/usr/bin/env python3
"""
record_waypoints.py — fixed-parameter version (no CLI args)

Records UGV waypoints from an AirSim-driven UE5 session while you manually drive.
Writes each line as:
    X Y Z T
with six decimal places, matching your existing Husky trajectory text files.

Press Ctrl+C to stop recording.
"""

import time
import os
import sys
from math import sqrt

import setup_path
import hercules as airsim

# ============================================================
# ====== USER PARAMETERS (edit these to your preferences) ====
# ============================================================
VEHICLE_NAME      = "Husky2"                # AirSim vehicle name
OUTFILE_PATH      = "/home/sgarimella34/multi-robot-coordination/trajectory_data/CSLAM_random_explore/test1_forest/Husky2_trajectory.txt" # Output trajectory file
APPEND_MODE       = False                   # If True, append instead of overwrite
SAMPLE_HZ         = 1.0                    # Sampling rate in Hz (e.g., 10 = every 0.1s)
DISTANCE_TRIGGER  = 4.0                     # Minimum meters moved before saving (0 disables)
ZERO_Z            = True                    # Force Z=0.0 in output
USE_ELAPSED_TIME  = True                   # If True, 4th column = elapsed seconds; else = counter
MAX_SAMPLES       = 0                       # Stop after N samples (0 = unlimited)
DECIMAL_PLACES    = 6                       # Decimal precision for output
# ============================================================


def dist3(a, b):
    return sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)


def main():
    mode = "a" if APPEND_MODE else "w"
    f = open(OUTFILE_PATH, mode)
    print(f"[record_waypoints] Writing to: {os.path.abspath(OUTFILE_PATH)} ({'append' if APPEND_MODE else 'overwrite'})")

    # Connect to AirSim
    client = airsim.CarClient()
    client.confirmConnection()
    client.enableApiControl(False, vehicle_name=VEHICLE_NAME)  # manual control, just reading pose

    period = 1.0 / SAMPLE_HZ if SAMPLE_HZ > 0 else 0.0
    next_t = time.time()
    start_time = time.time()
    last_saved_pos = None
    sample_idx = 0

    fmt = "{:." + str(DECIMAL_PLACES) + "f} " \
        + "{:." + str(DECIMAL_PLACES) + "f} " \
        + "{:." + str(DECIMAL_PLACES) + "f} " \
        + "{:." + str(DECIMAL_PLACES) + "f}\n"

    print(f"[record_waypoints] Vehicle: {VEHICLE_NAME}")
    print(f"[record_waypoints] Sampling: {SAMPLE_HZ} Hz | every_m: {DISTANCE_TRIGGER}")
    print(f"[record_waypoints] Z forced to 0: {ZERO_Z} | time_col: {'elapsed' if USE_ELAPSED_TIME else 'counter'}")
    print("[record_waypoints] Press Ctrl+C to stop.\n")

    try:
        while True:
            now = time.time()
            # wait until next sample if fixed rate enabled
            if period > 0.0 and now < next_t:
                time.sleep(max(0.0, next_t - now))
                continue
            if period > 0.0:
                next_t += period

            pose = client.simGetVehiclePose(vehicle_name=VEHICLE_NAME)
            pos = pose.position
            x, y, z = float(pos.x_val), float(pos.y_val), float(pos.z_val)
            if ZERO_Z:
                z = 0.0
            current_pos = (x, y, z)

            # skip if not moved enough
            if DISTANCE_TRIGGER > 0.0 and last_saved_pos is not None:
                if dist3(current_pos, last_saved_pos) < DISTANCE_TRIGGER:
                    continue

            # compute time column
            tcol = time.time() - start_time if USE_ELAPSED_TIME else float(sample_idx)

            # write to file
            f.write(fmt.format(x, y, z, tcol))
            f.flush()
            os.fsync(f.fileno())

            last_saved_pos = current_pos
            sample_idx += 1

            if MAX_SAMPLES > 0 and sample_idx >= MAX_SAMPLES:
                print(f"[record_waypoints] Reached MAX_SAMPLES={MAX_SAMPLES}. Stopping.")
                break

    except KeyboardInterrupt:
        print("\n[record_waypoints] Recording stopped by user.")
    finally:
        f.close()
        print(f"[record_waypoints] Saved {sample_idx} waypoints to {os.path.abspath(OUTFILE_PATH)}")


if __name__ == "__main__":
    main()
