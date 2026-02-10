#!/usr/bin/env python3

import setup_path
import hercules as airsim
import os

c = airsim.VehicleClient()
center = airsim.Vector3r(100, 100, 0)
# output_path = os.path.join(os.getcwd(), "AusLandscape_test2.binvox")
output_path = "/home/sgarimella34/Downloads/binvox-octomap/AusLandscape_test4.binvox"
# c.simCreateVoxelGrid(center, 100, 100, 100, 0.5, output_path) #works well; tried 300 but is sparse and breaks in viewvox executable
c.simCreateVoxelGrid(center, 100, 100, 100, 0.5, output_path) 