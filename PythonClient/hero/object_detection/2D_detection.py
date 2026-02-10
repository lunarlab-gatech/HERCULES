import setup_path 
import hercules as airsim
import cv2
import numpy as np 
import pprint

"""Works only for cylinders in static mesh form in block world, not dynamic Blueprint objects"""

# connect to the AirSim simulator
client = airsim.VehicleClient(port=41451)
client.confirmConnection()

# set camera name and image type to request images and detections
camera_name = "front_center"
image_type = airsim.ImageType.Scene

# set detection radius in [cm]
client.simSetDetectionFilterRadius(camera_name, image_type, 200 * 100) 
# add desired object name to detect in wild card/regex format NOTE THIS WORKS FOR STATIC MESH NAMES ONLY NOT THE NAME
# OF THE ACTOR IN THE UE5 WORLD OUTLINER
client.simAddDetectionFilterMeshName(camera_name, image_type, "Cylinder*") 
# client.simAddDetectionFilterMeshName(camera_name, image_type, "BP_SplineHuman_Type10*") 
# client.simAddDetectionFilterMeshName(camera_name, image_type, "BP_SplineHuman_Type10_C_UAID_E08F4CF5208A437A02_1596611129") 


while True:
    rawImage = client.simGetImage(camera_name, image_type)
    if not rawImage:
        continue
    png = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)
    cylinders = client.simGetDetections(camera_name, image_type)
    if cylinders:
        for cylinder in cylinders:
            s = pprint.pformat(cylinder)
            print("Cylinder: %s" % s)

            cv2.rectangle(png,(int(cylinder.box2D.min.x_val),int(cylinder.box2D.min.y_val)),(int(cylinder.box2D.max.x_val),int(cylinder.box2D.max.y_val)),(255,0,0),2)
            cv2.putText(png, cylinder.name, (int(cylinder.box2D.min.x_val),int(cylinder.box2D.min.y_val - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36,255,12))

    
    cv2.imshow("AirSim", png)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    elif cv2.waitKey(1) & 0xFF == ord('c'):
        client.simClearDetectionMeshNames(camera_name, image_type)
    elif cv2.waitKey(1) & 0xFF == ord('a'):
        client.simAddDetectionFilterMeshName(camera_name, image_type, "Cylinder*")
        # client.simAddDetectionFilterMeshName(camera_name, image_type, "BP_SplineHuman_Type10_C_UAID_E08F4CF5208A437A02_1596611129") 

cv2.destroyAllWindows() 