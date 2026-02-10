import setup_path
import hercules as airsim
import csv

client = airsim.VehicleClient(port=41451)
names = client.simListSceneObjects()
rows = []
for nm in names:
    seg = client.simGetSegmentationObjectID(nm)  # -1 if none set
    rows.append((nm, seg))
with open('/home/sgarimella34/multi-robot-coordination/HERCULES/csv_data/airsim_seg_mapping.csv', 'w', newline='') as f:
    csv.writer(f).writerows([('object_name', 'segmentation_id')] + rows)
