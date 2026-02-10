import setup_path                  # keep if you're using the local repo copy
import hercules as airsim
import numpy as np
import csv
import cv2
import re

# ---- user params ----
PORT          = 41451
CAMERA_NAME   = "front_center"
VEHICLE_NAME  = ""  # empty for default / single-vehicle setups

# where to dump / load the mesh-color CSV
CSV_FILENAME      = "/home/sgarimella34/multi-robot-coordination/" \
                    "HERCULES/csv_data/instance_segmentation_colormap.csv"
# where your UE label vs. mesh-name CSV lives
UE_LABEL_CSV_PATH = "/home/sgarimella34/multi-robot-coordination/" \
                    "HERCULES/csv_data/ue_label_vs_name.csv"

# if your raw segmentation image appears upside-down, set to True
FLIP_VERTICAL = False

# keywords to look for in the human-readable labels
KEYWORDS = ("human", "car", "truck", "sedan", "suv", "vehicle")

# regex for your spline-humans
REGEX = ".*BP_SplineHuman.*"

# --------------------------------

def seg_id_to_rgb(seg_id):
    # AirSim maps seg_id→(R,G,B) as (seg_id,0,0)
    return (seg_id, 0, 0)

# 1. connect
client = airsim.MultirotorClient(port=PORT)
client.confirmConnection()

# 2. list & filter scene objects
all_objs = client.simListSceneObjects()
pattern  = re.compile(REGEX)
matches  = [n for n in all_objs if pattern.match(n)]

print(f"\nFound {len(matches)} objects matching '{REGEX}':")
for n in matches:
    print(" ", n)

# 3. assign a shared seg-ID to all of them
SEG_ID = 200
print(f"\nSetting segmentation ID = {SEG_ID} on each human:")
for name in matches:
    ok = client.simSetSegmentationObjectID(name, SEG_ID, False)
    color = seg_id_to_rgb(SEG_ID)
    status = "OK" if ok else "FAIL"
    print(f" {status}: '{name}' → seg_id={SEG_ID}, color=RGB{color}")

# 4. flush the Python-side colormap cache in hercules
for attr in ("_segmentation_colormap",
             "segmentation_colormap",
             "_seg_colormap",
             "seg_colormap"):
    if hasattr(client, attr):
        delattr(client, attr)

# 5. now fetch a fresh colormap and print each spline-human’s actual ID
objects   = client.simListInstanceSegmentationObjects()
color_map = client.simGetSegmentationColorMap()  # Nx3 array

print("\n=== Current Segmentation IDs for BP_SplineHuman objects ===")
for name in matches:
    if name in objects:
        idx    = objects.index(name)
        seg_id = int(color_map[idx][0])  # R channel = seg_id
        print(f" '{name}' -> seg_id={seg_id}")
    else:
        print(f" '{name}' not found in simListInstanceSegmentationObjects()")

# 6. grab a full-HD segmentation image (1920×1080)
resp = client.simGetImages([
    airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.Segmentation, False, False)
], vehicle_name=VEHICLE_NAME)[0]

if resp.width == 0 or resp.height == 0:
    raise RuntimeError("Empty segmentation image.")

print(f"\nSeg image resolution: {resp.width}×{resp.height}")

# 7. decode to RGB numpy array
img1d   = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)
img_rgb = img1d.reshape(resp.height, resp.width, 3)
if FLIP_VERTICAL:
    img_rgb = np.flipud(img_rgb)

# 8. convert to BGR for OpenCV and display
img_bgr     = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
window_name = f"Segmentation (ID={SEG_ID})"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
cv2.resizeWindow(window_name, resp.width // 2, resp.height // 2)
cv2.imshow(window_name, img_bgr)
print("\nPress any key in the image window to exit.")
cv2.waitKey(0)
cv2.destroyAllWindows()
