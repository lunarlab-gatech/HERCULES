#!/usr/bin/env python3
# run_detector_oop.py (modified: no CLI args; switch vehicle in-code)

import setup_path
import hercules as airsim
from Hercules2D3DDetector import Hercules2D3DDetector as H

# =========================
# Inline configuration
# =========================
# Default remains the drone. To run the Husky UGV, set SELECTED_VEHICLE = "Husky1".
SELECTED_VEHICLE = "Husky1"   # "Drone1" (default) or "Husky1"
# SELECTED_VEHICLE = "Drone1"

# Optional: leave as None to use the detector's default camera name.
CAMERA_NAME_OVERRIDE = None   # e.g., "front_center" or None


def _configure_from_vehicle(vehicle_name: str):
    """Configure H.* based on the desired vehicle name (no CLI args)."""
    name_l = (vehicle_name or "").lower()

    # Infer platform & port from the chosen vehicle name
    if name_l.startswith("husky"):
        # Husky UGV on port 41452
        platform = "ugv"
        H.CLIENT_CLASS = airsim.CarClient
        H.PORT = 41452
    else:
        # Multirotor by default on port 41451
        platform = "drone"
        H.CLIENT_CLASS = airsim.MultirotorClient
        H.PORT = 41451

    # Pass vehicle down to the detector so LiDAR/poses use the same vehicle
    H.VEHICLE_NAME = vehicle_name

    # Optional camera override (keeps your defaults if None)
    if CAMERA_NAME_OVERRIDE:
        H.CAMERA_NAME = CAMERA_NAME_OVERRIDE

    print(
        f"[CFG] platform={platform} vehicle={H.VEHICLE_NAME} "
        f"client={H.CLIENT_CLASS.__name__} port={H.PORT} "
        f"camera={getattr(H, 'CAMERA_NAME', 'front_center')}"
    )


def main():
    _configure_from_vehicle(SELECTED_VEHICLE)
    H().run()


if __name__ == "__main__":
    main()
