#!/usr/bin/env python3
"""
Simple script to fetch and display the camera projection matrix from AirSim.
"""
import setup_path
import hercules as airsim
import numpy as np

def main():
    # Connect to the AirSim multirotor client
    client = airsim.MultirotorClient()
    client.confirmConnection()

    # Retrieve camera info for 'front_center' on vehicle 'Drone1'
    cam_info = client.simGetCameraInfo("front_center", vehicle_name="Drone1")

    # The projection matrix is provided as a flat list of 16 values (row-major)
    proj_list = cam_info.proj_mat.matrix
    # Reshape into a 4x4 NumPy array
    P = np.array(proj_list, dtype=float).reshape((4, 4))

    # Print the projection matrix
    print("Camera projection matrix (4x4):")
    print(P)

if __name__ == "__main__":
    main()
