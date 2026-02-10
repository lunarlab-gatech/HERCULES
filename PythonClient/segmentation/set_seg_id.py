#!/usr/bin/env python3

import setup_path
import hercules as airsim     


client = airsim.VehicleClient()
client.confirmConnection()

# Give every mesh whose name starts with "Crowd_" the ID 200
# client.simSetSegmentationObjectID("BP_CrowdCharacter*", 200, True)
check = client.simSetSegmentationObjectID("BP_SplineHuman*", 200, True)
print(check)
