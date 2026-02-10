import setup_path
import hercules as airsim

import sys
import time

drone_name = "Drone1"

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, drone_name)

print("arming the drone...")
client.armDisarm(True, drone_name)

state = client.getMultirotorState(vehicle_name=drone_name)
if state.landed_state == airsim.LandedState.Landed:
    print("taking off...")
    client.takeoffAsync(vehicle_name=drone_name).join()
else:
    client.hoverAsync(vehicle_name=drone_name).join()

time.sleep(1)

state = client.getMultirotorState(vehicle_name=drone_name)
if state.landed_state == airsim.LandedState.Landed:
    print("take off failed...")
    sys.exit(1)

# AirSim uses NED coordinates so negative axis is up.
# z of -5 is 5 meters above the original launch point.
z = -5
print("make sure we are hovering at {} meters...".format(-z))
client.moveToZAsync(z, 1, vehicle_name=drone_name).join()

# see https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo

# this method is async and we are not waiting for the result since we are passing timeout_sec=0.

print("flying on path...")
result = client.moveOnPathAsync([airsim.Vector3r(125,0,z),
                                airsim.Vector3r(125,-130,z),
                                airsim.Vector3r(0,-130,z),
                                airsim.Vector3r(0,0,z)],
                        12, 120,
                        airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), 20, 1, vehicle_name=drone_name).join()

time.sleep(2)
# drone will over-shoot so we bring it back to the start point before landing.
client.moveToPositionAsync(0,0,z,1, vehicle_name=drone_name).join()
client.hoverAsync(vehicle_name=drone_name).join()

print("landing...")
client.landAsync(vehicle_name=drone_name).join()

print("disarming...")
client.armDisarm(False, vehicle_name=drone_name)
client.enableApiControl(False, vehicle_name=drone_name)
print("done.")
