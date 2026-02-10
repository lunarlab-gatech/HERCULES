#!/bin/bash

# Absolute path to your executable
# EXECUTABLE_PATH=/home/sgarimella34/multi-robot-coordination/HERCULES/build_debug/output/bin/DroneWaypointControl
EXECUTABLE_PATH=/home/sgarimella34/multi-robot-coordination/HERCULES/build_release/output/bin/DroneWaypointControl
# EXECUTABLE_PATH=/home/dellg16ssg/multi-robot-coordination/HERCULES/build_release/output/bin/DroneWaypointControl

# Base path for waypoint files
# BEVP random explore motion
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/BEVP_random_explore"
# WAYPOINT_DIR="/home/dellg16ssg/multi-robot-coordination/trajectory_data/BEVP_random_explore"

# BEVP convoy motion
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/BEVP_convoy"
# WAYPOINT_DIR="/home/dellg16ssg/multi-robot-coordination/trajectory_data/BEVP_convoy"

# CSLAM random explore motion
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/CSLAM_random_explore"
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/CSLAM_random_explore/test_thruroad_dump"
# WAYPOINT_DIR="/home/dellg16ssg/multi-robot-coordination/trajectory_data/CSLAM_random_explore"

# BEVP DAIR-V2X data collection in custom city:
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/BEVP_customcity/"
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/BEVP_customcity2/"
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/BEVP_customcity3"
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/BEVP_customcity5"
WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/CSLAM_random_explore/test1_forest"

# Default number of drones if none specified
DEFAULT_NUM_DRONES=2

# Default velocity
VELOCITY=1.2 
# VELOCITY=5.0

# Default fly altitude. NOTE!!! This altitude is relative to the PlayerStart object's height in UE5, which is 1.5m above ground
# for DAIR-V2X-style inf camera flying in city env; so -6.0 is 
# FLY_ALTITUDE=-6.0 
# FLY_ALTITUDE=-10.0 
FLY_ALTITUDE=-24.0

# for under the canopy flying in AUsEnv
# FLY_ALTITUDE=-2.5  

# for BEVP in all envs
# FLY_ALTITUDE=-35.0

# Manually set return-home behavior (set to true or false)
DISABLE_RETURN_HOME=true

# Check if a velocity argument was provided via environment variable
if [[ -n "$WAYPOINT_VELOCITY" ]]; then
    VELOCITY=$WAYPOINT_VELOCITY
fi

# Check if altitude argument was provided via environment variable
if [[ -n "$FLY_ALTITUDE" ]]; then
    ALTITUDE=$FLY_ALTITUDE
else
    ALTITUDE=$FLY_ALTITUDE
fi

# Return home behavior flag
RETURN_HOME=true
if [[ "$DISABLE_RETURN_HOME" == "true" ]]; then
    RETURN_HOME=false
fi

# Store PIDs to wait on
PIDS=()

# Usage help
usage() {
    echo "Usage:"
    echo "  $0                     # Run all drones from Drone1 to Drone$DEFAULT_NUM_DRONES"
    echo "  $0 <num_drones>        # Run all drones from Drone1 to Drone<num_drones>"
    echo "  $0 Drone3              # Run only Drone3"
    echo ""
    echo "To set flight velocity, use:"
    echo "  WAYPOINT_VELOCITY=3.5 $0 Drone1"
    echo "To also set flight altitude, use:"
    echo "  WAYPOINT_VELOCITY=3.5 FLY_ALTITUDE=-50.0 $0 Drone1"
    echo "To disable return-to-home:"
    echo "  DISABLE_RETURN_HOME=true $0 Drone1"
    exit 1
}

# Launch function
launch_drone() {
    DRONE_NAME=$1
    WAYPOINT_FILE="$WAYPOINT_DIR/${DRONE_NAME}_trajectory.txt"

    echo "Launching $DRONE_NAME with waypoints from $WAYPOINT_FILE at velocity ${VELOCITY} m/s and altitude ${ALTITUDE} m"
    if [[ "$RETURN_HOME" == "true" ]]; then
        $EXECUTABLE_PATH "$DRONE_NAME" "$WAYPOINT_FILE" "$VELOCITY" "$ALTITUDE" &
    else
        $EXECUTABLE_PATH "$DRONE_NAME" "$WAYPOINT_FILE" "$VELOCITY" "$ALTITUDE" --no-return-home &
    fi
    PIDS+=($!)
}

# Determine what mode we're running
if [[ $# -eq 0 ]]; then
    # Default: run from 1 to DEFAULT_NUM_DRONES
    for i in $(seq 1 $DEFAULT_NUM_DRONES); do
        launch_drone "Drone$i"
    done
elif [[ $# -eq 1 ]]; then
    if [[ $1 =~ ^Drone[0-9]+$ ]]; then
        # Run only a specific drone like Drone3
        launch_drone "$1"
    elif [[ $1 =~ ^[0-9]+$ ]]; then
        # Run from Drone1 to DroneN
        for i in $(seq 1 $1); do
            launch_drone "Drone$i"
        done
    else
        usage
    fi
else
    usage
fi

# Wait for all launched drones to finish
for pid in "${PIDS[@]}"; do
    wait $pid
done

echo "Completed all requested drone flights."


# EXAMPLE USAGE
# Run Drone1 and Drone2 (default)
# ./run_drones.sh

# Run Drone1 through Drone5
# ./run_drones.sh 5

# Run only Drone3
# ./run_drones.sh Drone3

# Run Drone3 with 4.0 m/s velocity and -50.0 m altitude
# WAYPOINT_VELOCITY=4.0 FLY_ALTITUDE=-50.0 ./run_drones.sh Drone3

# Run Drone1 without return to home
# DISABLE_RETURN_HOME=true ./run_drones.sh Drone1
