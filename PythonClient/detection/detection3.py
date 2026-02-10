#!/usr/bin/env python3

"""Tests the method simAddSegmentationActor"""

import setup_path
import hercules as airsim
import cv2
import numpy as np

# 1) Connect with the multicopter client
# client = airsim.MultirotorClient()
# client = airsim.VehicleClient()
client = airsim.CarClient(ip="127.0.0.1", port=41452)


client.confirmConnection()
print("Connected!")

camera_name = "front_center"
image_type  = airsim.ImageType.Segmentation

# 2) Register all BP_CrowdCharacter* actors for segmentation
#    and assign them an instance ID (e.g. 201).
crowd = client.simListSceneObjects("BP_CrowdCharacter.*")
for actor_name in crowd:
    registered = client.simAddSegmentationActor(actor_name)
    if not registered:
        print(f"[WARN] failed to register {actor_name} for segmentation")
        continue
    client.simSetSegmentationObjectID(actor_name, 201, True)
print(f"Registered and ID'd {len(crowd)} crowd characters for segmentation")

# 3) Create and name your window once
window_name = "InstanceSeg"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

while True:
    # 4) Grab the latest segmentation frame
    raw = client.simGetImage(camera_name, image_type)
    if not raw:
        continue

    # 5) Decode as grayscale directly into an 8-bit single-channel image
    arr = airsim.string_to_uint8_array(raw)
    seg_img = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
    if seg_img is None:
        continue

    # 6) Apply a colormap for visualization
    color_seg = cv2.applyColorMap(seg_img, cv2.COLORMAP_JET)

    # 7) Resize for display and show it
    display = cv2.resize(color_seg, (1280, 720))
    cv2.imshow(window_name, display)

    # 8) Handle keypress
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
