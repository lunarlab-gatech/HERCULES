import setup_path
import hercules as airsim
import cv2
import numpy as np
import os
import time
import tempfile

# connect to the simulator
client = airsim.CarClient()
vnames = client.listVehicles()

client.confirmConnection()
client.enableApiControl(True)
print("API Control enabled: %s" % client.isApiControlEnabled())
car_controls = airsim.CarControls()
