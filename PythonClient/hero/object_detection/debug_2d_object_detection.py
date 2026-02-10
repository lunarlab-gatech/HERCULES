import setup_path                  # keep if you're using the local repo copy
import hercules as airsim
import numpy as np
import csv
import cv2

# ---- user params ----
PORT = 41451
CAMERA_NAME = "front_center"
VEHICLE_NAME = ""  # empty for default / single-vehicle setups

# where to dump / load the mesh-color CSV
CSV_FILENAME = "/home/sgarimella34/multi-robot-coordination/" \
               "HERCULES/csv_data/instance_segmentation_colormap.csv"

# where your UE label vs. mesh-name CSV lives
UE_LABEL_CSV_PATH = "/home/sgarimella34/multi-robot-coordination/" \
                    "HERCULES/csv_data/ue_label_vs_name.csv"

# where to save the depth frame (NumPy .npy format)
DEPTH_NPY_FILENAME = "/home/sgarimella34/multi-robot-coordination/" \
                     "HERCULES/csv_data/depth_frame.npy"

# if your raw segmentation image appears upside-down, set to True
FLIP_VERTICAL = False

# keywords to look for in the human-readable labels
KEYWORDS = ("human", "car", "truck", "sedan", "suv", "vehicle")

# maximum distance cutoff in meters
MAX_DISTANCE = 30.0

# ---------------------

# 1. connect
client = airsim.MultirotorClient(port=PORT)
client.confirmConnection()

# 2. dump the ID-name ↔ RGB colormap to CSV
objects   = client.simListInstanceSegmentationObjects()  # list of mesh ID names
color_map = client.simGetSegmentationColorMap()         # Nx3 array of RGB

with open(CSV_FILENAME, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["ObjectName", "R", "G", "B"])
    for idx, name in enumerate(objects):
        r, g, b = map(int, color_map[idx])
        writer.writerow([name, r, g, b])

# 3. reload that CSV into dictionaries
name_to_color = {}
color_to_name = {}
with open(CSV_FILENAME, newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        mesh = row["ObjectName"]
        rgb  = (int(row["R"]), int(row["G"]), int(row["B"]))
        name_to_color[mesh] = rgb
        color_to_name[rgb] = mesh

# 4. load UE label ↔ ID-name mapping
id_to_actor_label = {}
with open(UE_LABEL_CSV_PATH, newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        actor_label = row["actor_label"]
        mesh_name   = row["get_name"]
        id_to_actor_label[mesh_name] = actor_label

# 5. grab the segmentation and depth images at the same frozen timestamp
client.simPause(True)
responses = client.simGetImages([
    airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.Segmentation, False, False),
    airsim.ImageRequest(CAMERA_NAME, airsim.ImageType.DepthPlanar, True, False)
], vehicle_name=VEHICLE_NAME)
seg_resp, depth_resp = responses
client.simPause(False)

# check segmentation validity
if seg_resp.width == 0 or seg_resp.height == 0:
    print("Empty segmentation image.")
    exit(1)

# process segmentation image
img1d   = np.frombuffer(seg_resp.image_data_uint8, dtype=np.uint8)
img_rgb = img1d.reshape(seg_resp.height, seg_resp.width, 3)
if FLIP_VERTICAL:
    img_rgb = np.flipud(img_rgb)

# process depth image and save to .npy
depth1d = np.array(depth_resp.image_data_float, dtype=np.float32)
depth_img = depth1d.reshape(depth_resp.height, depth_resp.width)
if FLIP_VERTICAL:
    depth_img = np.flipud(depth_img)
np.save(DEPTH_NPY_FILENAME, depth_img)

# prepare a BGR copy of segmentation for drawing
display = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)

# 6. find which ID-names appear (by unique colors)
unique_colors = np.unique(img_rgb.reshape(-1, 3), axis=0)
visible_meshes = {
    color_to_name[tuple(c)]
    for c in unique_colors
    if tuple(c) in color_to_name and tuple(c) != (0, 0, 0)
}

# 7. filter for actor_labels matching keywords
candidates = []
for mesh in visible_meshes:
    label = id_to_actor_label.get(mesh)
    if label and any(kw in label.lower() for kw in KEYWORDS):
        candidates.append((mesh, label))

# 8. for each candidate, mask, apply depth cutoff, bbox, and draw
drawn_labels = []
for mesh, label in candidates:
    rgb = name_to_color[mesh]
    mask = ((img_rgb[:, :, 0] == rgb[0]) &
            (img_rgb[:, :, 1] == rgb[1]) &
            (img_rgb[:, :, 2] == rgb[2]))
    # apply distance cutoff
    mask &= (depth_img <= MAX_DISTANCE)
    mask_uint8 = (mask.astype(np.uint8) * 255)

    contours, _ = cv2.findContours(
        mask_uint8,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        continue

    # combine points to a single bounding box
    all_pts = np.vstack([cnt.reshape(-1, 2) for cnt in contours])
    x_min, y_min = all_pts.min(axis=0)
    x_max, y_max = all_pts.max(axis=0)

    # draw on `display`
    cv2.rectangle(display,
                  (x_min, y_min),
                  (x_max, y_max),
                  (255, 255, 255),
                  2)
    # label text just above box
    short = label if len(label) < 30 else label[:27] + "..."
    cv2.putText(display,
                short,
                (x_min, max(0, y_min - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA)

    drawn_labels.append(label)

# 9. show final result
win = "Filtered Instance Segmentation (<= {} m)".format(MAX_DISTANCE)
cv2.namedWindow(win, cv2.WINDOW_NORMAL)
cv2.resizeWindow(win, seg_resp.width, seg_resp.height)
cv2.imshow(win, display)
cv2.waitKey(0)
cv2.destroyAllWindows()

# 10. print what was found within cutoff
if drawn_labels:
    print("Detected objects within {} m:".format(MAX_DISTANCE))
    for lbl in drawn_labels:
        print(" -", lbl)
else:
    print("No matching objects within {} m found in current FOV.".format(MAX_DISTANCE))
