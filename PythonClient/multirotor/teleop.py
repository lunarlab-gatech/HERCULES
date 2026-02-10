#!/usr/bin/env python3

import setup_path 
import hercules as airsim
import sys
import time
import random
import select
import termios
import tty

# --- AirSim Client Setup ---
client = airsim.MultirotorClient()
client.confirmConnection()                      # verify RPC connection :contentReference[oaicite:3]{index=3}
client.enableApiControl(True)                   # gain API control :contentReference[oaicite:4]{index=4}
client.armDisarm(True)                          # arm motors :contentReference[oaicite:5]{index=5}
client.takeoffAsync().join()                    # take off :contentReference[oaicite:6]{index=6}
time.sleep(1)

# --- Terminal Raw Mode Setup ---
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setcbreak(fd)                               # raw mode for single-char reads :contentReference[oaicite:7]{index=7}

def get_key(timeout=0.1):
    """Read a single char from stdin with timeout; returns '' if none."""
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if r else ''

# --- Teleoperation Parameters ---
speed    = 1.0       # m/s translational speed :contentReference[oaicite:8]{index=8}
yaw_rate = 30        # deg/s rotational speed :contentReference[oaicite:9]{index=9}
duration = 0.05       # s per command interval :contentReference[oaicite:10]{index=10}

print("Teleop: WASD=XY, RF=Z, QE=Yaw, X=exit")

try:
    while True:
        key = get_key(duration)
        vx = vy = vz = yr = 0.0

        if key == 'w':      vx =  speed
        elif key == 's':    vx = -speed
        elif key == 'd':    vy =  speed
        elif key == 'a':    vy = -speed
        elif key == 'r':    vz =  speed
        elif key == 'f':    vz = -speed
        elif key == 'q':    yr = -yaw_rate
        elif key == 'e':    yr =  yaw_rate
        elif key.lower()=='x':
            print("Exit teleop…")
            break

        client.moveByVelocityAsync(
            vx, vy, vz, duration,
            yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yr)
        ).join()                                  # ensures sequential execution :contentReference[oaicite:11]{index=11}

finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  # restore terminal :contentReference[oaicite:12]{index=12}

# --- Shutdown Sequence ---
client.armDisarm(False)                       # disarm rotors :contentReference[oaicite:13]{index=13}
client.enableApiControl(False)                # release API control :contentReference[oaicite:14]{index=14}
print("Teleop session ended, client disconnected.")
