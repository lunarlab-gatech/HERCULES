#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Helper script to check the class -> color mapping in AirSim.

Usage:
    1. Set segmentation resolution to a low value in settings.json (10x10)
    2. Launch an Unreal scene and run this script
    3. Output `seg_colors.csv` will be written with a class -> RGB mapping
"""
import setup_path 
import hercules as airsim
import numpy as np
import csv

# client = airsim.MultirotorClient()
client = airsim.VehicleClient()
client.confirmConnection()

requests = airsim.ImageRequest("front_center", airsim.ImageType.Segmentation, False, False)

colors = {}
for cls_id in range(256):
    # map every asset to cls_id and extract the single RGB value produced
    client.simSetSegmentationObjectID(".*", cls_id, is_name_regex=True)
    response = client.simGetImages([requests])[0]
    img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
    img_rgb = img1d.reshape(response.height, response.width, 3)

    color = tuple(np.unique(img_rgb.reshape(-1, img_rgb.shape[-1]), axis=0)[0])
    print(f"{cls_id}\t{color}")
    colors[cls_id] = color

with open('/home/sgarimella34/multi-robot-coordination/HERCULES/csv_data/instance_segmentation_table.csv', 'w') as f:
    writer = csv.writer(f, delimiter=' ')
    for k, v in colors.items():
        writer.writerow([k] + list(v))