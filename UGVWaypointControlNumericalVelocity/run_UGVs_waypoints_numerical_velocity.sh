#!/bin/bash

# Absolute path to your UGV executable
# EXECUTABLE_PATH=/home/sgarimella34/multi-robot-coordination/HERCULES/build_debug/output/bin/UGVWaypointControlNumericalVelocity
EXECUTABLE_PATH=/home/sgarimella34/multi-robot-coordination/HERCULES/build_release/output/bin/UGVWaypointControlNumericalVelocity
# EXECUTABLE_PATH=/home/dellg16ssg/multi-robot-coordination/HERCULES/build_release/output/bin/UGVWaypointControlNumericalVelocity

# Base path for waypoint files
# BEVP random explore motion
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/BEVP_random_explore"
# WAYPOINT_DIR="/home/dellg16ssg/multi-robot-coordination/trajectory_data/BEVP_random_explore"

# BEVP convoy motion
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/BEVP_convoy"
# WAYPOINT_DIR="/home/dellg16ssg/multi-robot-coordination/trajectory_data/BEVP_convoy"

# CSLAM random explore motion
WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/CSLAM_random_explore"
# WAYPOINT_DIR="/home/sgarimella34/multi-robot-coordination/trajectory_data/CSLAM_random_explore/test_thruroad_dump"

# WAYPOINT_DIR="/home/dellg16ssg/multi-robot-coordination/trajectory_data/CSLAM_random_explore"

# Default number of UGVs if none specified
DEFAULT_NUM_UGVS=2

# Default linear speed (in m/s) when no individual speed is provided
DEFAULT_SPEED=1.4

# Default control-loop frequency (in Hz)
DEFAULT_CTRL_HZ=25

# Prefix for UGV names (adjust to match your naming convention)
PREFIX="Husky"

# Array to hold process IDs for launched instances
PIDS=()

# Usage help function
usage() {
    echo "Usage:"
    echo "  $0"
    echo "      # Run ${PREFIX}1 to ${PREFIX}${DEFAULT_NUM_UGVS} at ${DEFAULT_SPEED} m/s, ${DEFAULT_CTRL_HZ} Hz"
    echo "  $0 <num_ugvs>"
    echo "      # Run ${PREFIX}1 to ${PREFIX}<num_ugvs> at ${DEFAULT_SPEED} m/s, ${DEFAULT_CTRL_HZ} Hz"
    echo "  $0 <num_ugvs> <speed>"
    echo "      # Run ${PREFIX}1 to ${PREFIX}<num_ugvs> at <speed> m/s, ${DEFAULT_CTRL_HZ} Hz"
    echo "  $0 <num_ugvs> <speed> <ctrl_hz>"
    echo "      # Run ${PREFIX}1 to ${PREFIX}<num_ugvs> at <speed> m/s, <ctrl_hz> Hz"
    echo "  $0 ${PREFIX}3"
    echo "      # Run only ${PREFIX}3 at ${DEFAULT_SPEED} m/s, ${DEFAULT_CTRL_HZ} Hz"
    echo "  $0 ${PREFIX}3 <speed>"
    echo "      # Run only ${PREFIX}3 at <speed> m/s, ${DEFAULT_CTRL_HZ} Hz"
    echo "  $0 ${PREFIX}3 <speed> <ctrl_hz>"
    echo "      # Run only ${PREFIX}3 at <speed> m/s, <ctrl_hz> Hz"
    exit 1
}

# Launch a single UGV with given parameters
launch_one() {
    local name="$1"
    local speed="$2"
    local ctrl_hz="$3"
    local wp_file="${WAYPOINT_DIR}/${name}_trajectory.txt"
    echo "Launching ${name}: speed=${speed} m/s, ctrl=${ctrl_hz} Hz, waypoints=${wp_file}"
    "$EXECUTABLE_PATH" "$name" "$speed" "$wp_file" "$ctrl_hz" &
    PIDS+=($!)
}

# Determine the launch mode based on the number and type of arguments
case "$#" in
    0)
        speed=$DEFAULT_SPEED
        ctrl_hz=$DEFAULT_CTRL_HZ
        for i in $(seq 1 $DEFAULT_NUM_UGVS); do
            launch_one "${PREFIX}$i" "$speed" "$ctrl_hz"
        done
        ;;
    1)
        speed=$DEFAULT_SPEED
        ctrl_hz=$DEFAULT_CTRL_HZ
        if [[ $1 =~ ^[0-9]+$ ]]; then
            num_ugvs=$1
            for i in $(seq 1 $num_ugvs); do
                launch_one "${PREFIX}$i" "$speed" "$ctrl_hz"
            done
        else
            launch_one "$1" "$speed" "$ctrl_hz"
        fi
        ;;
    2)
        ctrl_hz=$DEFAULT_CTRL_HZ
        if [[ $1 =~ ^[0-9]+$ ]]; then
            num_ugvs=$1
            speed=$2
            for i in $(seq 1 $num_ugvs); do
                launch_one "${PREFIX}$i" "$speed" "$ctrl_hz"
            done
        else
            launch_one "$1" "$2" "$ctrl_hz"
        fi
        ;;
    3)
        if [[ $1 =~ ^[0-9]+$ ]]; then
            num_ugvs=$1
            speed=$2
            ctrl_hz=$3
            for i in $(seq 1 $num_ugvs); do
                launch_one "${PREFIX}$i" "$speed" "$ctrl_hz"
            done
        else
            launch_one "$1" "$2" "$3"
        fi
        ;;
    *)
        usage
        ;;
esac

# Wait for all launched UGV processes to complete
for pid in "${PIDS[@]}"; do
    wait "$pid"
done

echo "Completed all UGV waypoint missions."
