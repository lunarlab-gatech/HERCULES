# Running a UAV–UGV Team

This page runs HERCULES' heterogeneous mode (`"SimMode": "Hero"`) in the Blocks environment with two drones (`Drone1`, `Drone2`) and two Husky UGVs (`Husky1`, `Husky2`). You give each robot a trajectory, by teleoperation or with the trajectory planner, then replay all four together with the waypoint controllers while the ROS 2 bridges publish every robot's sensors, and optionally record the run as a dataset. See [Heterogeneous UAV–UGV Autonomy](heterogeneous_autonomy.md) for what the mode provides.

Hero mode is part of the HERCULES Unreal plugin. The upstream Cosys-AirSim packaged Blocks demo used in the [ROS 2 wrapper](ros_cplusplus.md) quick start does not include it, so this page builds the plugin into a Blocks project.

## Requirements

- Linux with an NVIDIA GPU and driver.
- [Unreal Engine 5.8](install_linux.md#install-unreal-engine) on Linux, or on [Windows](install_windows.md) with the ROS 2 bridges in WSL 2 (see [Simulator on Windows, ROS 2 in WSL](#simulator-on-windows-ros-2-in-wsl)).
- ROS 2 Jazzy on Ubuntu 24.04 or ROS 2 Lyrical on Ubuntu 26.04, for the ROS 2 bridges.

Run the commands below from the HERCULES repository root unless a step says otherwise. Replace `/path/to/UnrealEngine` with the folder that contains `Engine/`.

## 1. Build the plugin

```bash
./setup.sh
./build.sh --ue-root /path/to/UnrealEngine
```

`--ue-root` builds AirLib with Unreal Engine's bundled toolchain, which Ubuntu 24.04 and newer need (see [Install from Source on Linux](install_linux.md#build-hercules)). If you built before without `--ue-root`, delete `build_release/` first; CMake keeps the compiler a build folder was configured with. The build also produces the waypoint controllers in `build_release/output/bin/`.

## 2. Create the Blocks project

The repository holds the Blocks configuration and code, but not `Blocks.uproject` or its `Content/`. Copy them from Cosys-AirSim 5.8-v3.5.0, the Unreal Engine 5.8 release:

```bash
git clone --depth 1 --branch 5.8-v3.5.0 --filter=blob:none --sparse \
  https://github.com/Cosys-Lab/Cosys-AirSim /tmp/cosys-blocks
git -C /tmp/cosys-blocks sparse-checkout set Unreal/Environments/Blocks/Content
cp /tmp/cosys-blocks/Unreal/Environments/Blocks/Blocks.uproject Unreal/Environments/Blocks/
cp -r /tmp/cosys-blocks/Unreal/Environments/Blocks/Content Unreal/Environments/Blocks/
```

Then copy the freshly built plugin into the project:

```bash
(cd Unreal/Environments/Blocks && ./update_from_git.sh)
```

Rerun `update_from_git.sh` after every `./build.sh`.

## 3. Start the simulator

`ros2/settings/hero_blocks_team.json` declares the team. Drones spawn at the PlayerStart and 6 m to its right, and the Huskies 6 m ahead of each drone. Every robot carries a `front_center` camera (scene, planar and perspective depth, segmentation), `stereo_left`/`stereo_right` cameras, a 16-channel `LidarSensor1`, IMU, GPS, barometer and magnetometer. Keep the `Drone<N>`/`Husky<N>` names: the ROS bridges, run scripts and planner select vehicles by these prefixes.

Open the project with that settings file:

```bash
/path/to/UnrealEngine/Engine/Binaries/Linux/UnrealEditor \
  "$PWD/Unreal/Environments/Blocks/Blocks.uproject" \
  -settings="$PWD/ros2/settings/hero_blocks_team.json"
```

Answer **Yes** if the editor offers to build the `Blocks` module, and **No** if it offers to disable the AirSim plugin. When the editor has loaded, press **Play**. In Hero mode the simulator serves drones on RPC port 41451 and ground vehicles on 41452; its `ApiServerPort` setting does not apply. Press **Stop** and **Play** again to return the robots to their start positions.

## 4. Start the ROS 2 bridges

Build the ROS 2 workspace as described in the [ROS 2 wrapper](ros_cplusplus.md) page, then, from `HERCULES/ros2`:

```bash
source /opt/ros/jazzy/setup.bash   # or /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 launch airsim_ros_pkgs hercules_hero_team.launch.py
```

The launch file starts one bridge per RPC port, `hercules_uav_bridge` and `hercules_ugv_bridge`. Both publish below `/hercules_node/<Vehicle>/`, for example `/hercules_node/Husky1/lidar/points/LidarSensor1` and `/hercules_node/Drone2/front_center_Scene/image`. The drone bridge publishes `/clock` and offers `/hercules_node/<Drone>/takeoff`, `/land` and the `all_robots` services. Wait for `AirsimROSWrapper Initialized!` from both bridges.

To view all four robots, open the bundled RViz configuration from another sourced terminal:

```bash
rviz2 -d rviz2_configs/hercules_2UGVUAV.rviz
```

### Fly the drones from ROS 2

The drone bridge accepts velocity commands, so the keyboard controller or your own planner can fly the drones. The robots ignore commands until API control is enabled for them, so start the bridges with `enable_api_control:=True`, which enables it and arms every robot:

```bash
ros2 launch airsim_ros_pkgs hercules_hero_team.launch.py enable_api_control:=True
```

Then, from another sourced terminal, take off and fly `Drone1` with the keyboard (controls in [Keyboard flight](ros_cplusplus.md#keyboard-flight)):

```bash
ros2 service call /hercules_node/Drone1/takeoff airsim_interfaces/srv/Takeoff "{wait_on_last_task: true}"
ros2 run airsim_ros_pkgs uav_keyboard_teleop.py --bridge hercules --vehicle Drone1
```

| Topic | Type | Moves |
| --- | --- | --- |
| `/hercules_node/<Drone>/vel_cmd_body_frame` | `airsim_interfaces/msg/VelCmd` | that drone, relative to its heading |
| `/hercules_node/<Drone>/vel_cmd_world_frame` | `airsim_interfaces/msg/VelCmd` | that drone, in its start frame |
| `/hercules_node/all_robots/vel_cmd_body_frame`, `.../vel_cmd_world_frame` | `airsim_interfaces/msg/VelCmd` | every drone |
| `/hercules_node/group_of_robots/vel_cmd_body_frame`, `.../vel_cmd_world_frame` | `airsim_interfaces/msg/VelCmdGroup` | the drones in `vehicle_names` |

Velocities are in AirSim's NED convention: `linear.x` forward (north in the world frame), `linear.y` right (east), `linear.z` down, and `angular.z` the yaw rate in rad/s. Each message moves the drone for 0.05 s, so publish at 20 Hz or faster for continuous motion. With API control enabled, the Huskies take `airsim_interfaces/msg/CarControls` on `/hercules_node/<Husky>/car_cmd`.

## 5. Record a trajectory for each robot

The recorders use the `hercules_cosysairsim` client from this repository, which needs NumPy and `rpc-msgpack`:

```bash
python3 -m venv herculesvenv
source herculesvenv/bin/activate
pip install numpy rpc-msgpack
```

Record one robot at a time, with the terminal focused:

```bash
python PythonClient/hero/record_drone_waypoints_teleop.py --vehicle Drone1
python PythonClient/hero/record_drone_waypoints_teleop.py --vehicle Drone2
python PythonClient/hero/record_ugv_waypoints_teleop.py --vehicle Husky1
python PythonClient/hero/record_ugv_waypoints_teleop.py --vehicle Husky2
```

| | Drone | Husky |
| --- | --- | --- |
| Start | **T** take off | — |
| Move | **W/S/A/D** body-frame velocity, **R/F** altitude, **J/L** yaw | **W/S** throttle, **A/D** steer |
| Stop | **Space** or **H** hover | **Space** brake |
| Recording on/off | **C** | **V** |
| Quit | **Q** | **Q** |

Each recording is written to `trajectory_data/<Vehicle>_trajectory.txt`, overwriting an earlier one; pass `--out <file>` to write elsewhere. The full key lists are in each script's header. Press **Stop** and **Play** in the editor before replaying, so every robot starts from its spawn point.

## 6. Or plan the trajectories

Instead of recording, the trajectory planner can generate every robot's trajectory. With `flight_pattern:=RandomExplore` each robot explores the area on its own (coverage); with `flight_pattern:=Convoy` each drone follows the nearest Husky (leader–follower). The planner works on two occupancy grid maps, which `make_planning_maps.py` builds from the simulator.

**Build the maps.** With the editor playing and the Python environment from step 5 active, sample a 100 m square around the PlayerStart:

```bash
python PythonClient/hero/make_planning_maps.py --name blocks --size 100 \
  --settings ros2/settings/hero_blocks_team.json --drone-altitude 10
```

The simulator writes `trajectory_data/maps/blocks.binvox`, which can take a few minutes, and the script converts it into two maps in ROS `map_server` format:

- `blocks_ground.pgm`/`.yaml`: where the Huskies can drive. A cell is free when a Husky can reach it from its start, climbing at most `--step` (0.5 m) between cells and with nothing in the way up to `--ugv-height` (1 m). Box tops, walls and gaps in the floor are obstacles.
- `blocks_drone_10m.pgm`/`.yaml`: obstacles within `--drone-margin` (2 m) of the drone altitude, 10 m above the PlayerStart.

Obstacles are black in the images. The sampled heights, `--z-min` (-5 m) to `--z-max` (15 m) above the PlayerStart, must include the floor and the drone altitude; `--res` sets the cell size (0.5 m). To convert a capture again with different settings, without the simulator, pass `--binvox trajectory_data/maps/blocks.binvox` instead of `--name` and `--size`.

**Plan.** The planner does not need the simulator. From `HERCULES/ros2`, in a sourced terminal:

```bash
ros2 launch octomap_server plan_team_trajectories.launch.py \
  settings_file:=$PWD/settings/hero_blocks_team.json \
  ground_map:=$PWD/../trajectory_data/maps/blocks_ground.yaml \
  drone_map:=$PWD/../trajectory_data/maps/blocks_drone_10m.yaml \
  output_folder:=$PWD/../trajectory_data \
  square_size:=100 flight_pattern:=RandomExplore
```

The planner plans the Huskies on the ground map, then the drones on the drone map, logs `Trajectory saved to ...` for each robot, and exits. Each `trajectory_data/<Vehicle>_trajectory.txt` is overwritten and, like a recording, is relative to the robot's start.

| Argument | Default | Meaning |
| --- | --- | --- |
| `flight_pattern` | `RandomExplore` | `RandomExplore` (coverage) or `Convoy` (each drone follows the nearest Husky) |
| `trajectory_length` | `100.0` | metres per robot |
| `square_size` | `100.0` | side of the planning area in metres; use the maps' `--size` |
| `drone_altitude` | `10.0` | metres above the PlayerStart; use the drone map's `--drone-altitude` |

A vehicle entry in the settings file can override the pattern and length with `"FlightPattern"` and `"TrajectoryLength"`. If the planner cannot continue a robot's trajectory, for example a Husky boxed in by obstacles, it logs `stuck after ... m` and that trajectory ends early. To look at the result in RViz, start RViz first and add `exit_when_done:=false`: the planner publishes each robot's path once, as `nav_msgs/Path` on `/<Vehicle>_trajectory` in the `map` frame, and the maps stay on `/planning/ground_map` and `/planning/drone_map` until **Ctrl-C**.

Replay planned trajectories as in the next step, with `USE_WAYPOINT_Z=true` so the drones fly at the altitude they were planned for:

```bash
./UGVWaypointControl/run_UGVs_waypoints.sh 2 &
USE_WAYPOINT_Z=true ./DroneWaypointControl/run_drones_waypoints.sh 2
```

## 7. Replay the team

Start both controller sets; the UGV command runs in the background:

```bash
./UGVWaypointControl/run_UGVs_waypoints.sh 2 &
./DroneWaypointControl/run_drones_waypoints.sh 2
```

The drones take off, fly their recorded paths at 0.75 m/s and 4 m above the start, hover, and land. The Huskies track theirs at 0.29 m/s. These defaults are the speeds HERCULES' dataset collection uses. The scripts read trajectories from `trajectory_data/` and take these overrides:

| Setting | Example |
| --- | --- |
| Trajectory folder | `WAYPOINT_DIR=/path/to/trajectories` |
| Simulator address, for a simulator on another machine or on Windows with the scripts in WSL | `HERCULES_HOST=172.28.160.1` |
| Drone speed and altitude (NED, negative is up) | `WAYPOINT_VELOCITY=1.5 FLY_ALTITUDE=-6` |
| Fly each waypoint's recorded altitude | `USE_WAYPOINT_Z=true` |
| Return to the start before landing | `DISABLE_RETURN_HOME=false` |
| UGV count, speed and control rate | `./UGVWaypointControl/run_UGVs_waypoints.sh 2 0.3 20` |

To record the run, start `ros2 bag record -o hero_run -a` in another sourced terminal before replaying.

## 8. Generate a dataset

The dataset pipeline replays the trajectories itself while logging every robot's sensors in lockstep with the simulation, then records camera–IMU calibration manoeuvres and writes world-frame poses and synthetic IMU data. `dataset_pipeline/configs/blocks_team.yaml` sets it up for this team. With the editor playing, the robots at their start positions and the Python environment from step 5 active:

```bash
pip install scipy opencv-python pillow pyyaml
python dataset_pipeline/generate_dataset.py dataset_pipeline/configs/blocks_team.yaml \
  --stages collect,calibrate,post
```

Do not start the run scripts as well. The dataset is written to `~/hercules_datasets/blocks_team_01`; change `sequence` in a copy of the config for each new run, and set `use_waypoint_z: true` for planned trajectories. Add `--dry-run` to check the paths and print the commands without running them. The `labels` stage, which exports the segmentation label map, needs extra setup in the editor; see `dataset_pipeline/README.md`.

## Simulator on Windows, ROS 2 in WSL

The simulator can run on Windows, from the editor or a packaged build, while the ROS 2 bridges, run scripts and Python clients run in WSL 2:

- In WSL's default NAT networking, Windows is the WSL default gateway (`ip route show default`), and the simulator must accept connections from outside Windows' loopback: set `"LocalHostIp": "0.0.0.0"` in the settings file. With mirrored networking (`networkingMode=mirrored` in `.wslconfig`), `127.0.0.1` works from WSL as well.
- Windows Firewall asks whether to allow the simulator (`UnrealEditor.exe`, or the packaged `<Project>.exe`) the first time it listens. Allow it, or WSL cannot connect to ports 41451 and 41452.
- Pass the Windows address to the bridges, run scripts and clients, for example `ros2 launch airsim_ros_pkgs hercules_hero_team.launch.py host_ip:=172.28.160.1`, `HERCULES_HOST=172.28.160.1 ./DroneWaypointControl/run_drones_waypoints.sh 2` and `MultirotorClient(ip="172.28.160.1", port=41451)`.

## What has been tested

With Unreal Engine 5.8.3 on Windows 11 (Visual Studio 2026, MSVC 14.51) and the clients in WSL 2 (Ubuntu 26.04, ROS 2 Lyrical):

- `build.cmd` builds AirLib, rpclib 2.3.1 and MavLinkCom, and the Blocks project builds and packages for Win64 with the plugin. `./build.sh --ue-toolchain` with Epic's clang 20 toolchain for Unreal Engine 5.8 builds the Linux AirLib, rpclib and MavLinkCom libraries against its glibc 2.28 sysroot and libc++.
- In Blocks with `hero_blocks_team.json`, from the editor (`-game`) and from the packaged build, both RPC servers list all four robots; every robot returns scene, planar depth, segmentation, ThermalIR and NightVision images and LiDAR, IMU and GPS data; `Drone1` takes off and climbs 5 m; `Husky1` drives 10 m.
- The team launch file brings up both bridges against the simulator with `/clock` at 20 Hz, IMU at 100 Hz and every robot's camera and LiDAR topics; velocity commands fly `Drone2` and `car_cmd` drives `Husky2`.
- A Husky driven off the map below the level's KillZ returns to its start pose instead of being destroyed.

Earlier, without a running Unreal Engine, on Ubuntu 24.04 and 26.04:

- `hero_blocks_team.json` loads in HERCULES' settings parser as Hero mode with the `UGVPawn` Husky.
- Against a stand-in RPC server for the two Hero ports, `DroneWaypointControl` completes its takeoff, path and landing sequence, and velocity commands on each of the drone topics above reach the server as `moveByVelocity` calls for the right drones, rotated into the drone's heading for the body-frame topics.
- `make_planning_maps.py` converts a synthetic voxel grid with boxes, a tower, a curb, a raised platform, an overhang, a canopy and a gap in the floor into the expected ground and drone maps, and `plan_team_trajectories.launch.py` plans all four robots on them with both patterns: every trajectory starts at its robot's spawn point and stays clear of the obstacles on its map, and in `Convoy` each drone tracks its Husky's path.
- With the Blocks config, `generate_dataset.py --dry-run` resolves every path in the repository, and the `post` stage processes synthetic odometry into world-frame poses and synthetic IMU data.

The waypoint controllers, the planner and the dataset pipeline have not yet been run against the simulator.
