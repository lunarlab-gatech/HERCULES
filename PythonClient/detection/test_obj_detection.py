#!/usr/bin/env python3

import setup_path
import hercules as airsim
import cv2
import numpy as np


camera_name = "front_center"
image_type = airsim.ImageType.Scene

client = airsim.MultirotorClient()
client.confirmConnection()

client.simSetDetectionFilterRadius(camera_name, image_type, 80 * 100) # in [cm]
client.simAddDetectionFilterMeshName(camera_name, image_type, "Cylinder*") 
client.simGetDetections(camera_name, image_type)
detections = client.simClearDetectionMeshNames(camera_name, image_type)