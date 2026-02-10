#!/usr/bin/env python3

import setup_path
import hercules as airsim
import cv2
import numpy as np

# 1) Connect with the multicopter client
# client = airsim.MultirotorClient()
client = airsim.CarClient(ip="127.0.0.1", port=41452)
client.confirmConnection()

camera_name = "front_center"
image_type  = airsim.ImageType.Scene

# 2) Configure detection filters
client.simSetDetectionFilterRadius(camera_name, image_type, 200 * 100)  # 200 m
client.simClearDetectionMeshNames(camera_name, image_type)
# client.simAddDetectionFilterMeshName( camera_name, image_type, "BP_CrowdCharacter*" )
client.simAddDetectionFilterMeshName( camera_name, image_type, "BP_SplineHuman*" )

client.simAddDetectionFilterMeshName( camera_name, image_type, "Car*" )
client.simAddDetectionFilterMeshName( camera_name, image_type, "Sportscar*" )


# 3) Create and name your window once
window_name = "Detection"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

while True:
    raw = client.simGetImage(camera_name, image_type)
    if not raw:
        # no image yet, just retry
        continue

    # 4) Decode as BGR
    arr = airsim.string_to_uint8_array(raw)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        continue

    # 5) Get detections and draw boxes
    detections = client.simGetDetections(camera_name, image_type)
    if detections:
        for d in detections:
            x1 = int(d.box2D.min.x_val)
            y1 = int(d.box2D.min.y_val)
            x2 = int(d.box2D.max.x_val)
            y2 = int(d.box2D.max.y_val)
            cv2.rectangle(img, (x1, y1), (x2, y2), (0,0,255), 2)
            cv2.putText(
                img, d.name, (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0,0,255), 1, cv2.LINE_AA
            )

    # 6) Resize for display and show it once
    # small = cv2.resize(img, (0,0), fx=0.25, fy=0.25)
    # cv2.imshow(window_name, small)
    
    display = cv2.resize(img, (1280, 720))
    cv2.imshow(window_name, display)

    # 7) Handle keypress
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
