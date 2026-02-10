#!/usr/bin/env python3

import setup_path
import hercules as airsim

import time
import numpy as np
import os
import tempfile
import pprint

# Use below in settings.json with Blocks environment
"""
{
    "SettingsVersion": 1.2,
    "SimMode": "Hero",

    "Vehicles": {
        "Husky1": {
          "VehicleType": "CPHusky",
          "X": 0, "Y": 0, "Z": -2
        },
        "Drone1": {
          "VehicleType": "SimpleFlight",
          "X": 0, "Y": 0, "Z": -5,
      "Yaw": 90
        }
  }
}

"""

# connect to the AirSim simulator
client_1 = airsim.MultirotorClient(port=41451)
client_1.confirmConnection()
client_1.enableApiControl(True, "Drone1")
client_1.armDisarm(True, "Drone1")

client_2 = airsim.CarClient(port=41452)
client_2.confirmConnection()
client_2.enableApiControl(True, "Husky1")
ugv_controls1 = airsim.CarControls()

f1 = client_1.takeoffAsync(vehicle_name="Drone1")
car_state1 = client_2.getCarState("Husky1")
print("Husky1: Speed %d, Gear %d" % (car_state1.speed, car_state1.gear))
f1.join()
state1 = client_1.getMultirotorState(vehicle_name="Drone1")
s = pprint.pformat(state1)
print("Drone1: State: %s" % s)

# test motion of car and drone
f1 = client_1.moveToPositionAsync(-5, 5, -10, 5, vehicle_name="Drone1")
ugv_controls1.throttle = 0.2
ugv_controls1.steering = 0.5
client_2.setCarControls(ugv_controls1, "Husky1")
print("Husky1: Go Forward")
f1.join()

time.sleep(2)
airsim.wait_key('Press any key to take images')
# get camera images from the car
responses1 = client_1.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
    airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)], vehicle_name="Drone1")  #scene vision image in uncompressed RGBA array
print('Drone1: Retrieved images: %d' % len(responses1))
responses2 = client_2.simGetImages([
	airsim.ImageRequest("0", airsim.ImageType.Segmentation),  #depth visualization image
	airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)], "Husky1")  #scene vision image in uncompressed RGBA array
print('Husky1: Retrieved images: %d' % (len(responses2)))


tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
print ("Saving images to %s" % tmp_dir)
try:
    os.makedirs(tmp_dir)
except OSError:
    if not os.path.isdir(tmp_dir):
        raise

for idx, response in enumerate(responses1 + responses2):
    filename = os.path.join(tmp_dir, str(idx))

    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
    elif response.compress: #png format
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
    else: #uncompressed array
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        # temporarily commenting out for debugging purposes
        # img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) #get numpy array
        # img_rgba = img1d.reshape(response.height, response.width, 4) #reshape array to 4 channel image array H X W X 4
        # img_rgba = np.flipud(img_rgba) #original image is flipped vertically
        # img_rgba[:,:,1:2] = 100 #just for fun add little bit of green in all pixels
        # airsim.write_png(os.path.normpath(filename + '.greener.png'), img_rgba) #write to png

airsim.wait_key('Press any key to reset to original state')

client_1.armDisarm(False, "Drone1")
client_1.reset()
client_2.reset()

# that's enough fun for now. let's quit cleanly
client_1.enableApiControl(False, "Drone1")
client_2.enableApiControl(False, "Husky1")
