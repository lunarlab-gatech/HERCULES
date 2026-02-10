import setup_path
import hercules as airsim
import sys
import time

z = 35  # target altitude (in meters)
if len(sys.argv) > 1:
    z = float(sys.argv[1])

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

landed = client.getMultirotorState().landed_state
if landed == airsim.LandedState.Landed:
    print("Taking off...")
    client.takeoffAsync().join()
else:
    print("Already flying...")
    client.hoverAsync().join()

print(f"Rising to target altitude: {z} meters")
client.moveToZAsync(-z, 3).join()
client.hoverAsync().join()

print(f"Hovering at {z} meters. Control remains active.")
