import setup_path 

import hercules as airsim
client = airsim.MultirotorClient()
client.confirmConnection()

# Try applying a segmentation ID to a known object
success = client.simSetSegmentationObjectID("Drone1", 42, False)
print("Success?", success)
