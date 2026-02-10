#!/usr/bin/env python3
"""
ugv_teleop.py — Terminal keyboard teleop for AirSim/HERCULES UGVs (Husky).

Controls (hold or tap keys):
  W/S : throttle up/down
  A/D : steer left/right
  SPACE: full brake (hold)
  C   : handbrake toggle
  R   : reset vehicle
  P   : toggle global sim pause
  1/2 : quick throttle presets (0.25 / 0.5)
  0   : throttle 0 (idle)
  [ ] : decrease/increase steering center trim (for wheel alignment)
  - = : decrease/increase max steering limit
  , . : decrease/increase throttle step
  G/H : downshift/upshift (manual gear toggle with M)
  M   : toggle manual gear
  Q   : quit (or Ctrl-C)

Usage:
  python3 ugv_teleop.py                # defaults: Husky1 @ port 41452
  python3 ugv_teleop.py --vehicle Husky2 --port 41452 --hz 30

Notes:
- Works with both hercules and upstream airsim Python APIs.
- Sends commands at a fixed rate; keypresses adjust the current command.
"""

import sys, time, os, argparse, math, select, tty, termios, contextlib

# --- Try HERCULES first (your setup), fall back to upstream 'airsim' ---
import setup_path
import hercules as airsim

@contextlib.contextmanager
def raw_keyboard():
    """Put stdin into raw, non-blocking mode, restore on exit."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def kbhit(timeout=0.0):
    """Return a single character if available within timeout seconds; else None."""
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        ch = os.read(sys.stdin.fileno(), 1)
        try:
            return ch.decode('utf-8', errors='ignore')
        except Exception:
            return None
    return None

def clamp(x, lo, hi): return max(lo, min(hi, x))

def print_help():
    print(__doc__.split("Controls")[1].split("Usage")[0].strip())

def main():
    ap = argparse.ArgumentParser(description="Terminal teleop for AirSim/HERCULES UGVs")
    ap.add_argument("--vehicle", default="Husky1", help="Vehicle name")
    ap.add_argument("--port", type=int, default=41452, help="RPC port for CarClient")
    ap.add_argument("--hz", type=float, default=30.0, help="Command update rate (Hz)")
    ap.add_argument("--steer-max", type=float, default=0.75, help="Initial max |steering|")
    ap.add_argument("--throttle-step", type=float, default=0.05, help="Throttle increment per keypress")
    ap.add_argument("--steer-step", type=float, default=0.05, help="Steering increment per keypress")
    args = ap.parse_args()

    client = airsim.CarClient(port=args.port)
    print(f"Connecting CarClient(port={args.port}) ...")
    client.confirmConnection()
    print("Connected.")

    vehicle = args.vehicle
    client.enableApiControl(True, vehicle_name=vehicle)
    print(f"API control: ENABLED for {vehicle}")

    # If the sim is globally paused (common when using a stepper/collector), unpause:
    try:
        if client.simIsPaused():
            print("Sim is paused → unpausing now (press 'P' to toggle).")
            client.simPause(False)
    except Exception:
        # Older APIs may not have simIsPaused; ignore.
        pass

    # Command state
    throttle = 0.0       # [0..1]
    steering = 0.0       # [-1..1]
    brake = 0.0          # [0..1]
    handbrake = False
    is_manual_gear = False
    manual_gear = 1
    steer_center_trim = 0.0
    steer_max = clamp(args.steer_max, 0.1, 1.0)
    throttle_step = clamp(args.throttle_step, 0.01, 0.5)
    steer_step = clamp(args.steer_step, 0.01, 0.5)

    rate_dt = 1.0 / max(1e-6, args.hz)
    last_status = 0.0

    def send():
        controls = airsim.CarControls()
        controls.throttle = clamp(throttle, 0.0, 1.0)
        controls.steering = clamp(steering + steer_center_trim, -steer_max, steer_max)
        controls.brake = clamp(brake, 0.0, 1.0)
        controls.handbrake = handbrake
        controls.is_manual_gear = is_manual_gear
        controls.manual_gear = manual_gear
        controls.gear_immediate = True
        client.setCarControls(controls, vehicle_name=vehicle)

    def status():
        nonlocal last_status
        now = time.time()
        if now - last_status > 0.25:
            last_status = now
            state = client.getCarState(vehicle_name=vehicle)
            v = state.speed
            print(f"\rthr={throttle:4.2f} brk={brake:4.2f} str={steering:5.2f} "
                  f"trim={steer_center_trim:5.2f} | smax={steer_max:4.2f} "
                  f"{'HB' if handbrake else '  '} "
                  f"{'M' if is_manual_gear else 'A'}G{manual_gear} | "
                  f"speed={v:5.2f} m/s   ", end="", flush=True)

    print("Teleop ready. Press 'Q' to quit. Press '?' for help.")
    print_help()

    try:
        with raw_keyboard():
            while True:
                t0 = time.time()
                ch = kbhit(timeout=0.0)
                while ch:
                    c = ch.lower()

                    if c == 'q':
                        raise KeyboardInterrupt

                    elif c == 'w':
                        throttle = clamp(throttle + throttle_step, 0.0, 1.0)
                        brake = 0.0
                    elif c == 's':
                        throttle = clamp(throttle - throttle_step, 0.0, 1.0)

                    elif c == 'a':
                        steering = clamp(steering - steer_step, -1.0, 1.0)
                    elif c == 'd':
                        steering = clamp(steering + steer_step, -1.0, 1.0)

                    elif c == ' ':
                        brake = 1.0
                        throttle = 0.0

                    elif c == 'c':
                        handbrake = not handbrake

                    elif c == 'r':
                        print("\nResetting vehicle...")
                        client.reset()
                        time.sleep(0.2)
                        client.enableApiControl(True, vehicle_name=vehicle)
                        throttle = steering = brake = 0.0
                        handbrake = False

                    elif c == 'p':
                        try:
                            paused = client.simIsPaused()
                            client.simPause(not paused)
                            print(f"\nSim pause → {not paused}")
                        except Exception:
                            print("\nSim pause toggle not supported by this API build.")

                    elif c == '1':
                        throttle, brake = 0.25, 0.0
                    elif c == '2':
                        throttle, brake = 0.50, 0.0
                    elif c == '0':
                        throttle, brake = 0.0, 0.0

                    elif c == '[':
                        steer_center_trim = clamp(steer_center_trim - 0.01, -0.5, 0.5)
                    elif c == ']':
                        steer_center_trim = clamp(steer_center_trim + 0.01, -0.5, 0.5)
                    elif c == '-':
                        steer_max = clamp(steer_max - 0.05, 0.1, 1.0)
                    elif c == '=':
                        steer_max = clamp(steer_max + 0.05, 0.1, 1.0)

                    elif c == ',':
                        throttle_step = clamp(throttle_step - 0.01, 0.01, 0.5)
                    elif c == '.':
                        throttle_step = clamp(throttle_step + 0.01, 0.01, 0.5)

                    elif c == 'm':
                        is_manual_gear = not is_manual_gear
                        print(f"\nManual gear: {is_manual_gear}")
                    elif c == 'g':  # downshift
                        manual_gear = max(-1, manual_gear - 1)
                    elif c == 'h':  # upshift
                        manual_gear = min(6, manual_gear + 1)

                    elif ch == '?':
                        print()
                        print_help()

                    ch = kbhit(timeout=0.0)

                send()
                status()

                # Small decay so steering recenters gradually when not pressing keys
                steering *= 0.95

                # Sleep to maintain loop rate
                dt = time.time() - t0
                time.sleep(max(0.0, rate_dt - dt))

    except KeyboardInterrupt:
        print("\nExiting teleop...")

    finally:
        try:
            # Safe stop
            controls = airsim.CarControls()
            controls.throttle = 0.0
            controls.brake = 1.0
            controls.steering = 0.0
            controls.handbrake = False
            controls.is_manual_gear = False
            client.setCarControls(controls, vehicle_name=vehicle)
            client.enableApiControl(False, vehicle_name=vehicle)
            print("API control: DISABLED, vehicle stopped.")
        except Exception:
            pass

if __name__ == "__main__":
    main()
