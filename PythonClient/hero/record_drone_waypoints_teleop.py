#!/usr/bin/env python3
"""
record_drone_waypoints_teleop.py

Single-script keyboard teleop + waypoint recorder for a Cosys-AirSim multirotor.

- Teleop sends commands at TELEOP_HZ.
- Recorder writes waypoints as:  X Y Z T
  with fixed decimal places.

Key behaviors:
- WASD are BODY-FRAME (relative to drone heading), not world axes.
  W = forward, S = back, A = left, D = right in the drone's current yaw frame.
- Altitude is held using moveByVelocityZAsync(vx, vy, z_hold).
- R/F adjust the altitude hold target.

Coordinate notes:
- AirSim multirotor pose is NED (z positive down).
- If OUTPUT_Z_UP=True, logging flips Z to be up-positive.

Controls:
  W/S : forward/back (body frame)          [m/s]
  A/D : left/right (body frame)            [m/s]
  R/F : up/down (adjust altitude hold)     [m]
  J/L : yaw left/right (rate)              [deg/s]
  SPACE or 0 : stop XY+yaw and hold altitude
  H   : hover (also locks altitude)
  T   : takeoff + climb to TAKEOFF_ALT_M above start
  G   : land
  P   : toggle global sim pause
  C   : toggle recording on/off
  , . : decrease/increase XY speed step
  K/I : decrease/increase max XY speed
  U/O : decrease/increase altitude step (R/F)
  [ ] : decrease/increase yaw step
  Q   : quit (or Ctrl+C)

Press Ctrl+C to exit.
"""

import time
import os
import sys
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
VEHICLE_NAME      = "Drone1"
RPC_PORT          = 41451

# None = <repo>/trajectory_data/<VEHICLE_NAME>_trajectory.txt, where the waypoint run
# scripts look by default. --vehicle / --out on the command line override both.
OUTFILE_PATH      = None
APPEND_MODE       = False

# Recording
RECORD_ENABLED    = True
SAMPLE_HZ         = 2.0          # waypoint sampling rate (Hz); 0 disables recording
DISTANCE_TRIGGER  = 1.0          # meters moved before saving (0 disables)
USE_ELAPSED_TIME  = True         # 4th col = elapsed seconds; else = counter
MAX_SAMPLES       = 0            # stop after N samples (0 = unlimited)
DECIMAL_PLACES    = 6
OUTPUT_Z_UP       = True         # True: output Z as up-positive (flip NED z)

# Teleop command loop
TELEOP_HZ         = 30.0

# Takeoff behavior
TAKEOFF_ALT_M     = 4.0         # meters above starting altitude (positive up)
TAKEOFF_SPEED_MPS = 2.0          # climb speed used by moveToZAsync

# Speed tuning
VEL_XY_STEP_MPS   = 0.5          # increment per keypress (m/s)
YAW_STEP_DEGPS    = 10.0         # increment per keypress (deg/s)

MAX_VEL_XY_MPS    = 8.0
MAX_YAW_DEGPS     = 90.0

# Altitude step tuning (R/F modifies Z hold)
ALT_STEP_M        = 0.5          # meters per keypress
ALT_STEP_MIN_M    = 0.1
ALT_STEP_MAX_M    = 5.0
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
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def quat_to_yaw_rad(q):
    w = float(q.w_val)
    x = float(q.x_val)
    y = float(q.y_val)
    z = float(q.z_val)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def main():
    # File setup
    mode = "a" if APPEND_MODE else "w"
    outdir = os.path.dirname(OUTFILE_PATH)
    if outdir:
        os.makedirs(outdir, exist_ok=True)

    f = open(OUTFILE_PATH, mode)
    print(f"[drone_teleop_record] Writing to: {os.path.abspath(OUTFILE_PATH)} ({'append' if APPEND_MODE else 'overwrite'})")

    # Connect
    client = airsim.MultirotorClient(port=RPC_PORT)
    print(f"[drone_teleop_record] Connecting MultirotorClient(port={RPC_PORT}) ...")
    client.confirmConnection()
    print("[drone_teleop_record] Connected.")

    # Take API control and arm
    client.enableApiControl(True, vehicle_name=VEHICLE_NAME)
    try:
        client.armDisarm(True, vehicle_name=VEHICLE_NAME)
    except Exception:
        pass

    # If paused, unpause
    try:
        if client.simIsPaused():
            client.simPause(False)
            print("[drone_teleop_record] Sim was paused -> unpaused.")
    except Exception:
        pass

    # Recorder state
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

    # Teleop state (BODY-FRAME intent)
    vx_body = 0.0
    vy_body = 0.0
    yaw_rate = 0.0

    vel_xy_step = float(VEL_XY_STEP_MPS)
    yaw_step = float(YAW_STEP_DEGPS)

    max_vxy = float(MAX_VEL_XY_MPS)
    max_yaw = float(MAX_YAW_DEGPS)

    alt_step_m = float(ALT_STEP_M)

    teleop_dt = 1.0 / max(1e-6, TELEOP_HZ)
    last_status = 0.0

    z_hold_ned = None
    z_takeoff_start_ned = None

    # Distance tracking (in logged coordinates)
    total_dist_m = 0.0
    last_dist_pos = None

    def get_pose():
        return client.simGetVehiclePose(vehicle_name=VEHICLE_NAME)

    def get_z_ned():
        pose = get_pose()
        return float(pose.position.z_val)

    def set_hold_to_current():
        nonlocal z_hold_ned
        z_hold_ned = get_z_ned()

    def body_to_world_xy(vx_b, vy_b):
        pose = get_pose()
        yaw = quat_to_yaw_rad(pose.orientation)

        cy = math.cos(yaw)
        sy = math.sin(yaw)

        vx_w = cy * vx_b - sy * vy_b
        vy_w = sy * vx_b + cy * vy_b
        return vx_w, vy_w

    def safe_hover(lock_altitude=True):
        nonlocal vx_body, vy_body, yaw_rate
        vx_body = 0.0
        vy_body = 0.0
        yaw_rate = 0.0
        if lock_altitude:
            set_hold_to_current()
        try:
            client.hoverAsync(vehicle_name=VEHICLE_NAME).join()
        except Exception:
            pass

    def send_cmd():
        nonlocal z_hold_ned
        dur = max(teleop_dt * 1.5, 0.05)

        if z_hold_ned is None:
            set_hold_to_current()

        vx_w, vy_w = body_to_world_xy(vx_body, vy_body)

        client.moveByVelocityZAsync(
            vx_w, vy_w, z_hold_ned,
            duration=dur,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate),
            vehicle_name=VEHICLE_NAME
        )

    def update_total_distance():
        nonlocal total_dist_m, last_dist_pos

        pose = get_pose()
        pos = pose.position
        x = float(pos.x_val)
        y = float(pos.y_val)
        z_ned = float(pos.z_val)
        z_out = -z_ned if OUTPUT_Z_UP else z_ned

        cur = (x, y, z_out)
        if last_dist_pos is None:
            last_dist_pos = cur
            return

        total_dist_m += dist3(cur, last_dist_pos)
        last_dist_pos = cur

    def maybe_record():
        nonlocal next_sample_t, last_saved_pos, sample_idx

        if not record_enabled or sample_period <= 0.0:
            return

        now = time.time()
        if now < next_sample_t:
            return
        next_sample_t += sample_period

        pose = get_pose()
        pos = pose.position
        x = float(pos.x_val)
        y = float(pos.y_val)
        z_ned = float(pos.z_val)

        z_out = -z_ned if OUTPUT_Z_UP else z_ned
        current_pos = (x, y, z_out)

        if DISTANCE_TRIGGER and DISTANCE_TRIGGER > 0.0 and last_saved_pos is not None:
            if dist3(current_pos, last_saved_pos) < float(DISTANCE_TRIGGER):
                return

        tcol = (time.time() - start_time) if USE_ELAPSED_TIME else float(sample_idx)

        f.write(fmt.format(x, y, z_out, tcol))
        f.flush()
        os.fsync(f.fileno())

        last_saved_pos = current_pos
        sample_idx += 1

        if MAX_SAMPLES and MAX_SAMPLES > 0 and sample_idx >= int(MAX_SAMPLES):
            print(f"\n[drone_teleop_record] Reached MAX_SAMPLES={MAX_SAMPLES}. Stopping.")
            raise KeyboardInterrupt

    def status_line():
        nonlocal last_status
        now = time.time()
        if now - last_status < 0.25:
            return
        last_status = now

        try:
            st = client.getMultirotorState(vehicle_name=VEHICLE_NAME)
            v = st.kinematics_estimated.linear_velocity
            speed = math.sqrt(float(v.x_val) ** 2 + float(v.y_val) ** 2 + float(v.z_val) ** 2)
        except Exception:
            speed = float("nan")

        rec = "REC" if record_enabled else "   "
        zmode = "Zup" if OUTPUT_Z_UP else "ZNED"
        zhold = z_hold_ned if z_hold_ned is not None else float("nan")

        print(
            f"\r[{rec}] vxB={vx_body:6.2f} vyB={vy_body:6.2f} | yaw_rate={yaw_rate:6.1f} deg/s"
            f" | step={vel_xy_step:4.2f} maxXY={max_vxy:4.1f} yawStep={yaw_step:4.1f}"
            f" | altStep={alt_step_m:4.2f} zHold(NED)={zhold:7.2f}"
            f" | dist={total_dist_m:9.2f} m"
            f" | {zmode} | speed={speed:6.2f} m/s   ",
            end="",
            flush=True
        )

    def do_takeoff_to_height():
        nonlocal z_takeoff_start_ned, z_hold_ned

        if z_takeoff_start_ned is None:
            z_takeoff_start_ned = get_z_ned()

        target_z_ned = z_takeoff_start_ned - float(TAKEOFF_ALT_M)

        print(f"\n[drone_teleop_record] Takeoff + climb to {TAKEOFF_ALT_M:.2f} m above start (target z_ned={target_z_ned:.2f}) ...")

        try:
            client.takeoffAsync(vehicle_name=VEHICLE_NAME).join()
        except Exception as e:
            print(f"[drone_teleop_record] takeoffAsync failed (continuing): {e}")

        try:
            client.moveToZAsync(target_z_ned, float(TAKEOFF_SPEED_MPS), vehicle_name=VEHICLE_NAME).join()
        except Exception as e:
            print(f"[drone_teleop_record] moveToZAsync failed: {e}")

        z_hold_ned = target_z_ned
        safe_hover(lock_altitude=False)

    print(f"[drone_teleop_record] Vehicle: {VEHICLE_NAME} | port: {RPC_PORT}")
    print(f"[drone_teleop_record] Recording: {'ON' if record_enabled else 'OFF'} | SAMPLE_HZ={SAMPLE_HZ} | every_m={DISTANCE_TRIGGER}")
    print(f"[drone_teleop_record] Output format: X Y Z T | Z mode: {'up-positive' if OUTPUT_Z_UP else 'NED (down-positive)'}")
    print(f"[drone_teleop_record] Takeoff height: {TAKEOFF_ALT_M} m above start (set TAKEOFF_ALT_M).")
    print("[drone_teleop_record] Press 'T' to takeoff, 'G' to land, 'Q' to quit.\n")

    try:
        with raw_keyboard():
            while True:
                loop_t0 = time.time()

                ch = kbhit(timeout=0.0)
                while ch:
                    c = ch.lower()

                    if c == "q":
                        raise KeyboardInterrupt

                    # Body-frame motion intent
                    elif c == "w":
                        vx_body = clamp(vx_body + vel_xy_step, -max_vxy, max_vxy)
                    elif c == "s":
                        vx_body = clamp(vx_body - vel_xy_step, -max_vxy, max_vxy)

                    elif c == "a":
                        vy_body = clamp(vy_body - vel_xy_step, -max_vxy, max_vxy)
                    elif c == "d":
                        vy_body = clamp(vy_body + vel_xy_step, -max_vxy, max_vxy)

                    # Up/down adjusts altitude hold target (NED z)
                    elif c == "r":
                        if z_hold_ned is None:
                            set_hold_to_current()
                        z_hold_ned -= alt_step_m
                    elif c == "f":
                        if z_hold_ned is None:
                            set_hold_to_current()
                        z_hold_ned += alt_step_m

                    # Yaw rate
                    elif c == "j":
                        yaw_rate = clamp(yaw_rate - yaw_step, -max_yaw, max_yaw)
                    elif c == "l":
                        yaw_rate = clamp(yaw_rate + yaw_step, -max_yaw, max_yaw)

                    # Stop/hover
                    elif c == " " or c == "0":
                        safe_hover(lock_altitude=True)
                    elif c == "h":
                        safe_hover(lock_altitude=True)

                    # Takeoff / land
                    elif c == "t":
                        do_takeoff_to_height()

                    elif c == "g":
                        print("\n[drone_teleop_record] Land...")
                        try:
                            client.landAsync(vehicle_name=VEHICLE_NAME).join()
                        except Exception as e:
                            print(f"[drone_teleop_record] landAsync failed: {e}")
                        set_hold_to_current()
                        vx_body = 0.0
                        vy_body = 0.0
                        yaw_rate = 0.0

                    # Pause toggle
                    elif c == "p":
                        try:
                            paused = client.simIsPaused()
                            client.simPause(not paused)
                            print(f"\n[drone_teleop_record] Sim pause -> {not paused}")
                        except Exception:
                            print("\n[drone_teleop_record] Sim pause toggle not supported by this API build.")

                    # Toggle recording
                    elif c == "c":
                        record_enabled = not record_enabled
                        print(f"\n[drone_teleop_record] Recording -> {'ON' if record_enabled else 'OFF'}")

                    # Step and limits tuning
                    elif c == ",":
                        vel_xy_step = clamp(vel_xy_step - 0.1, 0.1, 20.0)
                    elif c == ".":
                        vel_xy_step = clamp(vel_xy_step + 0.1, 0.1, 20.0)

                    elif c == "k":
                        max_vxy = clamp(max_vxy - 0.5, 0.5, 50.0)
                        vx_body = clamp(vx_body, -max_vxy, max_vxy)
                        vy_body = clamp(vy_body, -max_vxy, max_vxy)
                    elif c == "i":
                        max_vxy = clamp(max_vxy + 0.5, 0.5, 50.0)

                    elif c == "u":
                        alt_step_m = clamp(alt_step_m - 0.1, ALT_STEP_MIN_M, ALT_STEP_MAX_M)
                    elif c == "o":
                        alt_step_m = clamp(alt_step_m + 0.1, ALT_STEP_MIN_M, ALT_STEP_MAX_M)

                    elif c == "[":
                        yaw_step = clamp(yaw_step - 5.0, 5.0, 180.0)
                    elif c == "]":
                        yaw_step = clamp(yaw_step + 5.0, 5.0, 180.0)

                    ch = kbhit(timeout=0.0)

                # Send command, update distance, record, print status
                send_cmd()
                update_total_distance()
                maybe_record()
                status_line()

                # Gentle decay so it recenters if you stop pressing keys
                vx_body *= 0.95
                vy_body *= 0.95
                yaw_rate *= 0.90

                # Maintain loop rate
                dt = time.time() - loop_t0
                time.sleep(max(0.0, teleop_dt - dt))

    except KeyboardInterrupt:
        print("\n[drone_teleop_record] Exiting...")

    finally:
        try:
            safe_hover(lock_altitude=True)
        except Exception:
            pass

        try:
            client.armDisarm(False, vehicle_name=VEHICLE_NAME)
        except Exception:
            pass

        try:
            client.enableApiControl(False, vehicle_name=VEHICLE_NAME)
        except Exception:
            pass

        f.close()
        print(f"[drone_teleop_record] Saved {sample_idx} waypoints to {os.path.abspath(OUTFILE_PATH)}")
        print(f"[drone_teleop_record] Total distance traveled: {total_dist_m:.2f} m")


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
