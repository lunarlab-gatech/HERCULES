#!/usr/bin/env python3

import time
import setup_path
import hercules as airsim

def track_object_pose(client, object_name: str, interval: float = 0.1):
    """
    Polls simGetObjectPose for the given object at the specified interval
    and prints its position and orientation.
    """
    try:
        while True:
            pose = client.simGetObjectPose(object_name)
            if not pose:
                print(f"[Warning] Object '{object_name}' not found in the scene.")
            else:
                pos = pose.position
                ori = pose.orientation
                print(f"Position -> x: {pos.x_val:.2f}, y: {pos.y_val:.2f}, z: {pos.z_val:.2f} | "
                      f"Orientation -> w: {ori.w_val:.3f}, x: {ori.x_val:.3f}, "
                      f"y: {ori.y_val:.3f}, z: {ori.z_val:.3f}")
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nTracking interrupted by user.")


def main():
    # create a client (MultirotorClient works generically for simGet* calls)
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    # THE NAME of the object to track; change this to your target's name
    # object_name = "BP_SplineHuman_Type10_C_UAID_E08F4CF5208A437A02_1596611129"
    # object_name = "BP_VehicleAI_pickup_C_UAID_6C6E07132D49788102_1328099840"
    # object_name = "BP_SplineHuman_Mannequin2_C_UAID_E08F4CF5208A427A02_1146249951"
    # object_name = "BP_SplineHuman_Type10_C_UAID_6C6E07132D49C88102_1970519919"
    # object_name = "BP_SplineMotionHuman_C_UAID_6C6E07132D49588302_1105149321"
    object_name = "BP_VehicleAI_pickup_C_UAID_6C6E07132D49788102_1328099840"  # BP_VehicleAI_pickup4
    # object_name = "StaticMeshActor_UAID_E08F4CF5208AA07502_2022041209"  # Sportscar_3

    # polling interval in seconds
    interval = 0.5

    print(f"Starting to track '{object_name}' every {interval}s. Press Ctrl+C to stop.")
    track_object_pose(client, object_name, interval)


if __name__ == "__main__":
    main()
