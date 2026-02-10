#!/usr/bin/env python3

import setup_path
import hercules as airsim
import numpy as np         
import cv2                 

# client = airsim.VehicleClient()
# client = airsim.CarClient()
client = airsim.CarClient(ip="127.0.0.1", port=41452)

client.confirmConnection()

# 1 – see exactly what Unreal called the characters
crowd = client.simListSceneObjects("BP_CrowdCharacter.*")
print("Found:", crowd)

# 2 – (only if set InitialInstanceSegmentation = false)
for obj in crowd:
    client.simSetSegmentationObjectID(obj, 200, True)

# 3 – grab one segmentation frame (PNG bytes)
seg_png = client.simGetImage("front_center", airsim.ImageType.Segmentation)
assert seg_png, "Seg frame came back empty -> something above failed"

# 4 – decode and show it
img = cv2.imdecode(np.frombuffer(seg_png, np.uint8), cv2.IMREAD_UNCHANGED)
cv2.imshow("Instance Segmentation", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
cv2.waitKey(0)
cv2.destroyAllWindows()
