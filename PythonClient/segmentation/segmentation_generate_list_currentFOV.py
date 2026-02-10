#!/usr/bin/env python

import setup_path
import hercules as airsim
import csv
import numpy as np
from datetime import datetime

def main():
    # 1) Connect to HERCULES
    client = airsim.MultirotorClient()
    client.confirmConnection()

    # 2) Trigger proxy-mesh instance segmentation (uint8 RGB)
    print("Requesting segmentation image to initialize instance-colors...")
    seg_resp = client.simGetImages([
        airsim.ImageRequest(
            "front_center",
            airsim.ImageType.Segmentation,
            pixels_as_float=False,  # uint8 RGB
            compress=False
        )
    ])[0]

    # Convert raw bytes → H×W×3 uint8 array
    img1d = np.frombuffer(seg_resp.image_data_uint8, dtype=np.uint8)
    img_rgb = img1d.reshape(seg_resp.height, seg_resp.width, 3)

    # 3) Load the full instance-color map and object list
    print("Loading full segmentation color map and object names...")
    color_map = client.simGetSegmentationColorMap()    # N×3 uint8 array
    names     = client.simListInstanceSegmentationObjects()  # list of N names

    # 4) Build RGB→name lookup
    color_to_name = {
        tuple(color_map[i]): names[i]
        for i in range(len(names))
    }

    # 5) Extract only the colors actually visible
    unique_colors = np.unique(img_rgb.reshape(-1, 3), axis=0)
    timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    filename  = f"instance_segmentation_map_{timestamp}.csv"

    # 6) Write CSV: Name,R,G,B
    print(f"Writing {len(unique_colors)} visible colors to {filename}...")
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Name", "R", "G", "B"])
        for col in unique_colors:
            name = color_to_name.get(tuple(col), "UNKNOWN")
            writer.writerow([name, *col])

    print("Done.")

if __name__ == "__main__":
    main()
