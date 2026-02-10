import setup_path
import hercules as airsim

import sys
import time

def load_waypoints_from_file(file_path):
    waypoints = []
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if len(parts) >= 3:
                x, y, z = map(float, parts[:3])
                waypoints.append(airsim.Vector3r(x, y, z))
    return waypoints

drone_name = "Drone1"

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, drone_name)

print("Arming the drone...")
client.armDisarm(True, drone_name)

state = client.getMultirotorState(vehicle_name=drone_name)
if state.landed_state == airsim.LandedState.Landed:
    print("Taking off...")
    client.takeoffAsync(vehicle_name=drone_name).join()
else:
    client.hoverAsync(vehicle_name=drone_name).join()

time.sleep(1)

state = client.getMultirotorState(vehicle_name=drone_name)
if state.landed_state == airsim.LandedState.Landed:
    print("Takeoff failed...")
    sys.exit(1)

# AirSim uses NED coordinates so negative axis is up.
# z of -5 is 5 meters above the original launch point.
z = -5
print(f"Make sure we are hovering at {-z} meters...")
client.moveToZAsync(z, 1, vehicle_name=drone_name).join()

# Load waypoints from the text file
waypoints_file = '/home/sgarimella34/multi-robot-coordination/trajectory_data/Drone1_trajectory.txt'
waypoints = load_waypoints_from_file(waypoints_file)

if not waypoints:
    print("No valid waypoints found. Exiting...")
    sys.exit(1)

# Adjust waypoints' altitude to match the desired z value
adjusted_waypoints = [airsim.Vector3r(wp.x_val, wp.y_val, z) for wp in waypoints]

# Fly along the path
print("Flying on path...")
client.moveOnPathAsync(adjusted_waypoints, 3, 120,
                       airsim.DrivetrainType.ForwardOnly,
                       airsim.YawMode(False, 0), 20, 1,
                       vehicle_name=drone_name).join()

time.sleep(2)

# Return to the start point before landing
client.moveToPositionAsync(0, 0, z, 1, vehicle_name=drone_name).join()
client.hoverAsync(vehicle_name=drone_name).join()

print("Landing...")
client.landAsync(vehicle_name=drone_name).join()

print("Disarming...")
client.armDisarm(False, vehicle_name=drone_name)
client.enableApiControl(False, vehicle_name=drone_name)
print("Done.")
