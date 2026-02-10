#!/usr/bin/env python3

import setup_path                     # ensure hercules is on PYTHONPATH
import hercules as airsim
import numpy as np
from PIL import Image
from datetime import datetime

def main():
    # --- connect ---
    client = airsim.MultirotorClient()
    client.confirmConnection()

    layer  = "HumanAnnotation"
    camera = "front_center"

    # --- find your human mesh assets ---
    meshes = client.simGetMeshPositionVertexBuffers()
    spline_meshes = [m.name for m in meshes if "SplineHuman" in m.name]
    print("Found human mesh assets:", spline_meshes)

    # --- tag each mesh with blue ---
    for mesh_name in spline_meshes:
        client.simSetAnnotationObjectColor(
            layer,
            mesh_name,
            0,   # R
            0,   # G
            255, # B
            False  # exact match, not regex
        )

    # --- verify tagging ---
    tagged = client.simListAnnotationObjects(layer)
    print(f"Meshes in '{layer}':", tagged)

    # --- capture the annotation image ---
    resp = client.simGetImages([
        airsim.ImageRequest(
            camera,
            airsim.ImageType.Annotation,  # Annotation type
            pixels_as_float=False,        # uint8 RGB
            compress=False,               # raw bytes
            annotation_name=layer         # which layer
        )
    ])[0]

    # --- decode and save ---
    arr = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)
    rgb = arr.reshape(resp.height, resp.width, 3)
    img = Image.fromarray(rgb, 'RGB')
    filename = f"{layer}_{datetime.now():%Y%m%d_%H%M%S}.png"
    img.save(filename)
    print("Saved annotation image to:", filename)

if __name__ == "__main__":
    main()
