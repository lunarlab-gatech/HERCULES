#!/bin/bash

# Absolute path to your executable
# EXECUTABLE_PATH=/home/sgarimella34/multi-robot-coordination/Cosys-AirSim/build_debug/output/bin/DroneWaypointControl
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Built by ./build.sh; override with EXECUTABLE_PATH=... if yours lives elsewhere.
EXECUTABLE_PATH="${EXECUTABLE_PATH:-$SCRIPT_DIR/../build_release/output/bin/DroneWaypointControl}"
# EXECUTABLE_PATH=/home/dellg16ssg/multi-robot-coordination/Cosys-AirSim/build_release/output/bin/DroneWaypointControl

# Base path for waypoint files
# BEVP random explore motion
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/BEVP_random_explore"
# WAYPOINT_DIR="/home/dellg16ssg/multi-robot-coordination/trajectory_data/BEVP_random_explore"

# BEVP convoy motion
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/BEVP_convoy"
# WAYPOINT_DIR="/home/dellg16ssg/multi-robot-coordination/trajectory_data/BEVP_convoy"

# CSLAM random explore motion
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/CSLAM_random_explore/test1_dump/"
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/CSLAM_random_explore/test_thruroad_dump"
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/CSLAM_random_explore/city_test1/"
# WAYPOINT_DIR="/home/dellg16ssg/multi-robot-coordination/trajectory_data/CSLAM_random_explore"

# BEVP DAIR-V2X data collection in custom city:
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/BEVP_customcity/"
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/BEVP_customcity2/"
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/BEVP_customcity3"
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/BEVP_customcity5"
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/CSLAM_random_explore/test1_forest"
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/CSLAM_random_explore/customforest_test1/"

# SmallTown MG planning demo:
# Folder holding <Vehicle>_trajectory.txt files; override with WAYPOINT_DIR=...
WAYPOINT_DIR="${WAYPOINT_DIR:-$SCRIPT_DIR/../trajectory_data}"

# WAYPOINT_DIR="/home/sgarimella34/Documents/trajectory_editor_sandbox/"
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/test_drone/"

# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/CSLAM_random_explore/nuclearnev_test1/"
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/CSLAM_random_explore/ausenv_short_seq/"
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/CSLAM_random_explore/ausenv_road_seq/"

# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/CSLAM_random_explore/city_test2/"

# Default number of drones if none specified
DEFAULT_NUM_DRONES=2

# Default velocity
# VELOCITY=1.5 
VELOCITY=0.75
# VELOCITY=5

# Default fly altitude. NOTE!!! This altitude is relative to the PlayerStart object's height in UE5, which is 1.5m above ground
# for DAIR-V2X-style inf camera flying in city env;
# FLY_ALTITUDE=-6.0 #for ausenv around short seq
# FLY_ALTITUDE=-3.0 # for ausenv road seq
# FLY_ALTITUDE=-10.0 
# FLY_ALTITUDE=-24.0
FLY_ALTITUDE="${FLY_ALTITUDE:--4.0}" #for customforest

# for under the canopy flying in AUsEnv
# FLY_ALTITUDE=-2.5  

# for BEVP in all envs
# FLY_ALTITUDE=-35.0

# Manually set return-home behavior (set to true or false)
DISABLE_RETURN_HOME="${DISABLE_RETURN_HOME:-true}"

# Altitude mode: "fixed" flies every waypoint at FLY_ALTITUDE (legacy behavior);
# "waypoint" flies at each waypoint's own recorded Z (clamped to >=1m above origin).
# Override per-run with: USE_WAYPOINT_Z=true ./run_drones_waypoints.sh Drone1
ALTITUDE_MODE="fixed"
if [[ "$USE_WAYPOINT_Z" == "true" ]]; then
    ALTITUDE_MODE="waypoint"
fi

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

if [[ ! -x "$EXECUTABLE_PATH" ]]; then
    echo "ERROR: $EXECUTABLE_PATH not found. Build it with ./build.sh, or set EXECUTABLE_PATH."
    exit 1
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
    echo "To fly at each waypoint's recorded altitude instead of FLY_ALTITUDE:"
    echo "  USE_WAYPOINT_Z=true $0 Drone1"
    exit 1
}

# Launch function
launch_drone() {
    DRONE_NAME=$1
    WAYPOINT_FILE="$WAYPOINT_DIR/${DRONE_NAME}_trajectory.txt"

    EXTRA_FLAGS=()
    if [[ "$ALTITUDE_MODE" == "waypoint" ]]; then
        EXTRA_FLAGS+=("--use-waypoint-z")
        echo "Launching $DRONE_NAME with waypoints from $WAYPOINT_FILE at velocity ${VELOCITY} m/s using recorded waypoint altitudes"
    else
        echo "Launching $DRONE_NAME with waypoints from $WAYPOINT_FILE at velocity ${VELOCITY} m/s and altitude ${ALTITUDE} m"
    fi
    if [[ "$RETURN_HOME" == "true" ]]; then
        $EXECUTABLE_PATH "$DRONE_NAME" "$WAYPOINT_FILE" "$VELOCITY" "$ALTITUDE" "${EXTRA_FLAGS[@]}" &
    else
        $EXECUTABLE_PATH "$DRONE_NAME" "$WAYPOINT_FILE" "$VELOCITY" "$ALTITUDE" "${EXTRA_FLAGS[@]}" --no-return-home &
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
