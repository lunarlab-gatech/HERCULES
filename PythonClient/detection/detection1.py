import setup_path 
import hercules as airsim
import cv2
import numpy as np 
import pprint

# connect to the AirSim simulator
client = airsim.VehicleClient()
client.confirmConnection()

# set camera name and image type
# camera_name = "0"
camera_name = "front_center"
image_type  = airsim.ImageType.Scene

# 1) Clear any old mesh filters
client.simClearDetectionMeshNames(camera_name, image_type)  
# :contentReference[oaicite:0]{index=0}

# 2) Set detection radius to 1 m = 100 cm
client.simSetDetectionFilterRadius(camera_name, image_type, 200 * 100)  
# :contentReference[oaicite:1]{index=1}

# 3) Add a wildcard filter for “all” meshes
client.simAddDetectionFilterMeshName(camera_name, image_type, "BP_Kangaroo*")  
# :contentReference[oaicite:2]{index=2}

while True:
    # grab the raw scene image
    rawImage = client.simGetImage(camera_name, image_type)  
    if not rawImage:
        continue

    # decode & display
    png = cv2.imdecode(airsim.string_to_uint8_array(rawImage),
                       cv2.IMREAD_UNCHANGED)

    # pull all detections within 1 m
    detections = client.simGetDetections(camera_name, image_type)  
    # :contentReference[oaicite:3]{index=3}

    if detections:
        for det in detections:
            s = pprint.pformat(det)
            print(f"Detection: {s}")

            # draw 2D box & label
            cv2.rectangle(png,
                          (int(det.box2D.min.x_val), int(det.box2D.min.y_val)),
                          (int(det.box2D.max.x_val), int(det.box2D.max.y_val)),
                          (255, 0, 0), 2)
            cv2.putText(png,
                        det.name,
                        (int(det.box2D.min.x_val), int(det.box2D.min.y_val) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (36, 255, 12))

    cv2.imshow("AirSim", png)
    key = cv2.waitKey(1) & 0xFF
    if   key == ord('q'):  break
    elif key == ord('c'):  
        client.simClearDetectionMeshNames(camera_name, image_type)  
        # :contentReference[oaicite:4]{index=4}
    elif key == ord('a'):
        client.simAddDetectionFilterMeshName(camera_name,
                                             image_type,
                                             "*")  
        # :contentReference[oaicite:5]{index=5}

cv2.destroyAllWindows()
