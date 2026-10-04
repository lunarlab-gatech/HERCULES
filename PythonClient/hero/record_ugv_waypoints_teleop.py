#!/usr/bin/env python3
"""

record_ugv_waypoints_teleop.py

Single-script keyboard teleop + waypoint recorder for an AirSim/Cosys-AirSim UGV (Husky).

- Teleop sends CarControls at TELEOP_HZ.
- Recorder writes waypoints as:  X Y Z T  (fixed decimal places), matching your Husky files.
- Prints total distance traveled so far (meters) live in the status line.

Controls:
  W/S : throttle up/down (rate-limited so holding a key ramps smoothly)
  A/D : steer left/right
  SPACE: full brake (hold)
  C   : handbrake toggle
  R   : reset vehicle
  P   : toggle global sim pause
  1/2 : quick throttle presets (0.25 / 0.5)
  0   : throttle 0 (idle)
  [ ] : decrease/increase steering center trim
  - = : decrease/increase max steering limit
  , . : decrease/increase throttle step
  G/H : downshift/upshift (manual gear toggle with M)
  M   : toggle manual gear
  V   : toggle recording on/off
  Q   : quit (or Ctrl-C)

Notes:
- Uses CarClient(port=RPC_PORT).
- For logging, ZERO_Z=True forces z=0.0 (good for UGV 2D trajectories).
"""

import os
import sys
import time
import math
import select
import tty
import termios
import contextlib

import setup_path
import hercules_cosysairsim as airsim

# ============================================================
# ====== USER PARAMETERS (edit these to your preferences) ====
# ============================================================
VEHICLE_NAME      = "Husky1"
RPC_PORT          = 41452

# None = <repo>/trajectory_data/<VEHICLE_NAME>_trajectory.txt, where the waypoint run
# scripts look by default. --vehicle / --out on the command line override both.
OUTFILE_PATH      = None
APPEND_MODE       = False

# Teleop loop
TELEOP_HZ         = 30.0
STEER_MAX_INIT    = 0.75
THROTTLE_STEP_INIT= 0.01     # per bump
STEER_STEP        = 0.05
THROTTLE_REPEAT_HZ= 8.0      # holding W/S bumps this many times per second

# Recording
RECORD_ENABLED    = True
SAMPLE_HZ         = 1.0
DISTANCE_TRIGGER  = 4.0
ZERO_Z            = True
USE_ELAPSED_TIME  = True
MAX_SAMPLES       = 0
DECIMAL_PLACES    = 6

# Distance display
DISTANCE_MODE     = "live"   # "live" = integrate at TELEOP_HZ, "record" = only when a waypoint is written
# ============================================================


@contextlib.contextmanager
def raw_keyboard():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def kbhit(timeout=0.0):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        ch = os.read(sys.stdin.fileno(), 1)
        try:
            return ch.decode("utf-8", errors="ignore")
        except Exception:
            return None
    return None


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def dist3(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)


def main():
    # -------- File setup --------
    mode = "a" if APPEND_MODE else "w"
    outdir = os.path.dirname(OUTFILE_PATH)
    if outdir:
        os.makedirs(outdir, exist_ok=True)

    f = open(OUTFILE_PATH, mode)
    print(f"[ugv_teleop_record] Writing to: {os.path.abspath(OUTFILE_PATH)} ({'append' if APPEND_MODE else 'overwrite'})")

    # -------- Connect --------
    client = airsim.CarClient(port=RPC_PORT)
    print(f"[ugv_teleop_record] Connecting CarClient(port={RPC_PORT}) ...")
    client.confirmConnection()
    print("[ugv_teleop_record] Connected.")

    vehicle = VEHICLE_NAME
    client.enableApiControl(True, vehicle_name=vehicle)
    print(f"[ugv_teleop_record] API control: ENABLED for {vehicle}")

    # If paused, unpause
    try:
        if client.simIsPaused():
            print("[ugv_teleop_record] Sim is paused -> unpausing now (press 'P' to toggle).")
            client.simPause(False)
    except Exception:
        pass

    # -------- Teleop state --------
    throttle = 0.0
    steering = 0.0
    brake = 0.0
    handbrake = False
    is_manual_gear = False
    manual_gear = 1

    steer_center_trim = 0.0
    steer_max = clamp(float(STEER_MAX_INIT), 0.1, 1.0)
    throttle_step = clamp(float(THROTTLE_STEP_INIT), 0.001, 0.5)
    steer_step = clamp(float(STEER_STEP), 0.01, 0.5)

    throttle_repeat_hz = max(0.5, float(THROTTLE_REPEAT_HZ))
    throttle_repeat_dt = 1.0 / throttle_repeat_hz
    last_throttle_bump = 0.0

    teleop_dt = 1.0 / max(1e-6, float(TELEOP_HZ))
    last_status = 0.0

    # -------- Recorder state --------
    record_enabled = bool(RECORD_ENABLED)
    sample_period = 1.0 / SAMPLE_HZ if SAMPLE_HZ and SAMPLE_HZ > 0 else 0.0
    next_sample_t = time.time()
    start_time = time.time()
    last_saved_pos = None
    sample_idx = 0

    fmt = (
        "{:." + str(DECIMAL_PLACES) + "f} "
        + "{:." + str(DECIMAL_PLACES) + "f} "
        + "{:." + str(DECIMAL_PLACES) + "f} "
        + "{:." + str(DECIMAL_PLACES) + "f}\n"
    )

    # Distance tracking
    total_dist_m = 0.0
    last_dist_pos = None

    def get_pos_xyz():
        pose = client.simGetVehiclePose(vehicle_name=vehicle)
        pos = pose.position
        x, y, z = float(pos.x_val), float(pos.y_val), float(pos.z_val)
        if ZERO_Z:
            z = 0.0
        return (x, y, z)

    def send_controls():
        controls = airsim.CarControls()
        controls.throttle = clamp(throttle, 0.0, 1.0)
        controls.steering = clamp(steering + steer_center_trim, -steer_max, steer_max)
        controls.brake = clamp(brake, 0.0, 1.0)
        controls.handbrake = handbrake
        controls.is_manual_gear = is_manual_gear
        controls.manual_gear = manual_gear
        controls.gear_immediate = True
        client.setCarControls(controls, vehicle_name=vehicle)

    def update_total_distance():
        nonlocal total_dist_m, last_dist_pos
        cur = get_pos_xyz()
        if last_dist_pos is None:
            last_dist_pos = cur
            return
        total_dist_m += dist3(cur, last_dist_pos)
        last_dist_pos = cur

    def maybe_record():
        nonlocal next_sample_t, last_saved_pos, sample_idx, total_dist_m, last_dist_pos

        if not record_enabled or sample_period <= 0.0:
            return

        now = time.time()
        if now < next_sample_t:
            return
        next_sample_t += sample_period

        cur = get_pos_xyz()

        if DISTANCE_TRIGGER and float(DISTANCE_TRIGGER) > 0.0 and last_saved_pos is not None:
            if dist3(cur, last_saved_pos) < float(DISTANCE_TRIGGER):
                return

        tcol = (time.time() - start_time) if USE_ELAPSED_TIME else float(sample_idx)

        f.write(fmt.format(cur[0], cur[1], cur[2], tcol))
        f.flush()
        os.fsync(f.fileno())

        # If distance mode is "record", accumulate distance only when a waypoint is saved
        if DISTANCE_MODE == "record":
            if last_dist_pos is None:
                last_dist_pos = cur
            else:
                total_dist_m += dist3(cur, last_dist_pos)
                last_dist_pos = cur

        last_saved_pos = cur
        sample_idx += 1

        if MAX_SAMPLES and int(MAX_SAMPLES) > 0 and sample_idx >= int(MAX_SAMPLES):
            print(f"\n[ugv_teleop_record] Reached MAX_SAMPLES={MAX_SAMPLES}. Stopping.")
            raise KeyboardInterrupt

    def status_line():
        nonlocal last_status
        now = time.time()
        if now - last_status < 0.25:
            return
        last_status = now

        try:
            state = client.getCarState(vehicle_name=vehicle)
            speed = float(state.speed)
        except Exception:
            speed = float("nan")

        rec = "REC" if record_enabled else "   "
        print(
            f"\r[{rec}] thr={throttle:4.2f} brk={brake:4.2f} str={steering:5.2f} "
            f"trim={steer_center_trim:5.2f} | smax={steer_max:4.2f} "
            f"{'HB' if handbrake else '  '} "
            f"{'M' if is_manual_gear else 'A'}G{manual_gear} | "
            f"speed={speed:5.2f} m/s | dist={total_dist_m:9.2f} m   ",
            end="",
            flush=True
        )

    print(f"[ugv_teleop_record] Vehicle: {vehicle} | port: {RPC_PORT}")
    print(f"[ugv_teleop_record] Teleop: {TELEOP_HZ} Hz | Record: {'ON' if record_enabled else 'OFF'} | SAMPLE_HZ={SAMPLE_HZ} | every_m={DISTANCE_TRIGGER}")
    print("[ugv_teleop_record] Press 'Q' to quit. Press '?' for help. Press 'V' to toggle recording.\n")

    def print_help():
        print(__doc__.split("Controls:")[1].strip())

    try:
        with raw_keyboard():
            while True:
                loop_t0 = time.time()
                now = time.time()

                ch = kbhit(timeout=0.0)
                while ch:
                    c = ch.lower()
                    now = time.time()

                    if c == "q":
                        raise KeyboardInterrupt

                    # Throttle: rate-limited bumps
                    elif c == "w":
                        if now - last_throttle_bump >= throttle_repeat_dt:
                            throttle = clamp(throttle + throttle_step, 0.0, 1.0)
                            brake = 0.0
                            last_throttle_bump = now
                    elif c == "s":
                        if now - last_throttle_bump >= throttle_repeat_dt:
                            throttle = clamp(throttle - throttle_step, 0.0, 1.0)
                            last_throttle_bump = now

                    elif c == "a":
                        steering = clamp(steering - steer_step, -1.0, 1.0)
                    elif c == "d":
                        steering = clamp(steering + steer_step, -1.0, 1.0)

                    elif c == " ":
                        brake = 1.0
                        throttle = 0.0

                    elif c == "c":
                        handbrake = not handbrake

                    elif c == "r":
                        print("\n[ugv_teleop_record] Resetting vehicle...")
                        client.reset()
                        time.sleep(0.2)
                        client.enableApiControl(True, vehicle_name=vehicle)
                        throttle = steering = brake = 0.0
                        handbrake = False
                        # reset distance reference
                        last_dist_pos = None

                    elif c == "p":
                        try:
                            paused = client.simIsPaused()
                            client.simPause(not paused)
                            print(f"\n[ugv_teleop_record] Sim pause -> {not paused}")
                        except Exception:
                            print("\n[ugv_teleop_record] Sim pause toggle not supported by this API build.")

                    elif c == "1":
                        throttle, brake = 0.25, 0.0
                    elif c == "2":
                        throttle, brake = 0.50, 0.0
                    elif c == "0":
                        throttle, brake = 0.0, 0.0

                    elif c == "[":
                        steer_center_trim = clamp(steer_center_trim - 0.01, -0.5, 0.5)
                    elif c == "]":
                        steer_center_trim = clamp(steer_center_trim + 0.01, -0.5, 0.5)
                    elif c == "-":
                        steer_max = clamp(steer_max - 0.05, 0.1, 1.0)
                    elif c == "=":
                        steer_max = clamp(steer_max + 0.05, 0.1, 1.0)

                    elif c == ",":
                        throttle_step = clamp(throttle_step - 0.005, 0.001, 0.5)
                    elif c == ".":
                        throttle_step = clamp(throttle_step + 0.005, 0.001, 0.5)

                    elif c == "m":
                        is_manual_gear = not is_manual_gear
                        print(f"\n[ugv_teleop_record] Manual gear: {is_manual_gear}")
                    elif c == "g":
                        manual_gear = max(-1, manual_gear - 1)
                    elif c == "h":
                        manual_gear = min(6, manual_gear + 1)

                    elif c == "v":
                        record_enabled = not record_enabled
                        print(f"\n[ugv_teleop_record] Recording -> {'ON' if record_enabled else 'OFF'}")

                    elif ch == "?":
                        print()
                        print_help()

                    ch = kbhit(timeout=0.0)

                # Apply
                send_controls()

                # Distance tracking
                if DISTANCE_MODE == "live":
                    update_total_distance()

                # Record if time
                maybe_record()

                # Status line
                status_line()

                # Small decay so steering recenters
                steering *= 0.95

                # Maintain loop rate
                dt = time.time() - loop_t0
                time.sleep(max(0.0, teleop_dt - dt))

    except KeyboardInterrupt:
        print("\n[ugv_teleop_record] Exiting...")

    finally:
        try:
            controls = airsim.CarControls()
            controls.throttle = 0.0
            controls.brake = 1.0
            controls.steering = 0.0
            controls.handbrake = False
            controls.is_manual_gear = False
            client.setCarControls(controls, vehicle_name=vehicle)
            client.enableApiControl(False, vehicle_name=vehicle)
        except Exception:
            pass

        try:
            f.close()
        except Exception:
            pass

        print(f"[ugv_teleop_record] Saved {sample_idx} waypoints to {os.path.abspath(OUTFILE_PATH)}")
        print(f"[ugv_teleop_record] Total distance traveled: {total_dist_m:.2f} m")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Teleop a vehicle and record its waypoint trajectory.")
    parser.add_argument("--vehicle", default=VEHICLE_NAME, help="vehicle name (default: %(default)s)")
    parser.add_argument("--out", default=OUTFILE_PATH,
                        help="trajectory file (default: <repo>/trajectory_data/<vehicle>_trajectory.txt)")
    args = parser.parse_args()
    VEHICLE_NAME = args.vehicle
    OUTFILE_PATH = args.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..", "trajectory_data",
        f"{VEHICLE_NAME}_trajectory.txt")
    main()
