#!/usr/bin/env python3

import setup_path
import hashlib
import json
import os
import hercules as airsim
import numpy as np
import matplotlib.pyplot as plt

def stable_id(name: str) -> int:
    """MD5‐hash the name → mod 254 → +1 yields IDs in [1-254]."""
    h = hashlib.md5(name.encode("utf-8")).hexdigest()
    return (int(h, 16) % 254) + 1

def main():
    # Connect to HERCULES/HERCULES RPC server
    client = airsim.MultirotorClient()
    client.confirmConnection()

    # 1) Retrieve all scene object names
    all_objs = client.simListSceneObjects(".*")

    # 2) Tag every object for segmentation
    mapping = {}
    for obj_name in all_objs:
        obj_id = stable_id(obj_name)
        success = client.simSetSegmentationObjectID(
            obj_name, obj_id, is_name_regex=False
        )
        if not success:
            print(f"[WARN] Failed to tag '{obj_name}'")
        mapping[obj_name] = obj_id

    # 3) Persist the mapping
    out_dir = os.path.dirname(__file__)  # script’s directory
    mapping_path = os.path.join(out_dir, "segmentation_mapping.json")
    with open(mapping_path, "w") as f:
        json.dump(mapping, f, indent=2)
    print(f"Tagged {len(all_objs)} objects. Mapping saved to {mapping_path}")

    # 4) Capture one segmentation image from camera 0
    imgs = client.simGetImages([
        airsim.ImageRequest(0, airsim.ImageType.Segmentation, False, False)
    ])
    if not imgs:
        print("[ERROR] No image returned")
    else:
        raw = imgs[0]

        # 5) Convert to NumPy array
        img1d = np.frombuffer(raw.image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape(raw.height, raw.width, 3)
        img_rgb = np.flipud(img_rgb)

        # 6) Save segmentation image
        img_path = os.path.join(out_dir, "segmentation_view.png")
        plt.imsave(img_path, img_rgb)
        print(f"Segmentation image saved to {img_path}")

    # 7) Cleanly release API control
    client.enableApiControl(False)
    print("API control disabled, client session closed.")

if __name__ == "__main__":
    main()
