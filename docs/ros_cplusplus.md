# airsim_ros_pkgs

A ROS2 wrapper over the HERCULES C++ client library. All coordinates and data are in the right-handed coordinate frame of the ROS standard and not in NED except for geo points.
The workspace builds on ROS 2 Jazzy (Ubuntu 24.04) and ROS 2 Lyrical (Ubuntu 26.04).

## Build

- Build HERCULES as per the instructions.

- Source your ROS distribution (replace `jazzy` with `lyrical` when using Lyrical) and enter the ROS workspace from the HERCULES repository root:

```shell
source /opt/ros/jazzy/setup.bash
cd ros2
```

All remaining commands run inside `HERCULES/ros2`. If your terminal is already there, skip `cd ros2`; use `src` as the dependency path, not `ros2/src`.

- Set up rosdep once, if it is not already installed and initialized:

```shell
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
```

- Install dependencies and build the workspace. The build runs only if dependency installation succeeds:

```shell
rosdep install --from-paths src --ignore-src -y --rosdistro "$ROS_DISTRO" &&
  colcon build --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Running

After a successful build, start the simulator and press Play if using the Unreal editor. The simulator must expose its RPC server (default `localhost:41451`) before the ROS wrapper can initialize.

For a basic test without installing Unreal Editor, the repository includes a downloader for the upstream Cosys-AirSim Blocks packaged Linux demo. From `HERCULES/ros2`, run it in a separate terminal:

```shell
(cd ../docker &&
  bash download_blocks_env_binary.sh &&
  bash Blocks_packaged_Linux_52_32/Linux/Blocks.sh -windowed -ResX=1280 -ResY=720)
```

This demo provides the upstream test environment. HERCULES-specific environments and features require the HERCULES simulator plugin.

The bridge detects the mode of a single default vehicle when the simulator's settings omit `SimMode`, as can happen after interactive vehicle selection in the packaged demo. For custom vehicle configurations, set `SimMode` explicitly in the simulator's `settings.json` (for example, `"SimMode": "Car"`) and restart the simulator after editing the file.

```shell
source install/setup.bash &&
  ros2 launch airsim_ros_pkgs airsim_node.launch.py
```

For a simulator on another machine or port, pass `host_ip:=<simulator-ip>` and `host_port:=<rpc-port>` to the launch command. Repeated `Waiting for connection - X...` output means the wrapper has not connected to that endpoint. Check the simulator's `RpcEnabled`, `LocalHostIp`, and `ApiServerPort` settings and any firewall between the machines.

The AirLib connection wait does not observe ROS shutdown, so Ctrl-C during this wait can require launch to escalate to SIGKILL. Do not launch after a failed build: an existing installation can still run an older executable.

For the packaged Blocks demo's default car, open RViz in another terminal from `HERCULES/ros2`:

```shell
source install/setup.bash &&
  rviz2 -d rviz2_configs/blocks_car.rviz
```

This configuration shows a grid, coordinate frames, and the car's odometry trail in the `world` frame. The view follows `PhysXCar/odom_local`. RViz must use the same `ROS_DOMAIN_ID` and middleware settings as the bridge. Camera images and LiDAR point clouds require those sensors to be configured in the simulator before starting the bridge.

### Blocks car with sensors

The included `ros2/settings/blocks_car_sensors.json` enables a 16-channel LiDAR, three front RGB cameras, front-center planar depth, IMU, GPS, barometer, magnetometer, and a forward distance sensor. Camera images are 640 by 360 pixels. The camera and range sensor mounts are forward of the car body, and the LiDAR is raised above it to avoid self-obstruction. Settings use metres in the vehicle's NED frame: positive X is forward, positive Y is right, and negative Z is up.

Stop an existing Blocks simulator and bridge before using this configuration. Run these commands in separate terminals, each from `HERCULES/ros2`.

Start the simulator with an explicit settings path; this leaves `~/Documents/AirSim/settings.json` unchanged:

```shell
bash ../docker/Blocks_packaged_Linux_52_32/Linux/Blocks.sh \
  -settings="$(pwd)/settings/blocks_car_sensors.json" \
  -windowed -ResX=1280 -ResY=720
```

Once the simulator is running, start the bridge with 10 Hz camera and LiDAR requests:

```shell
source install/setup.bash &&
  ros2 launch airsim_ros_pkgs airsim_node.launch.py \
    host_ip:=127.0.0.1 image_update_interval:=0.1 lidar_update_interval:=0.1
```

Open the RViz configuration:

```shell
source install/setup.bash &&
  rviz2 -d rviz2_configs/blocks_car.rviz
```

RViz displays the LiDAR cloud, front RGB and depth images, IMU axes and acceleration, and odometry. Enable the `Left RGB` and `Right RGB` displays to view the other cameras. Sensor topics are below `/airsim_node/PhysXCar/`:

| Topic suffix | Data |
| --- | --- |
| `lidar/points/lidar` | `sensor_msgs/msg/PointCloud2` |
| `front_center_Scene/image` | RGB image |
| `front_center_DepthPlanar/image` | Float depth image in metres |
| `front_left_Scene/image`, `front_right_Scene/image` | Left and right RGB images |
| `imu/imu` | `sensor_msgs/msg/Imu` |
| `gps/gps` | `sensor_msgs/msg/NavSatFix` |
| `altimeter/barometer` | `airsim_interfaces/msg/Altimeter` |
| `magnetometer/magnetometer` | `sensor_msgs/msg/MagneticField` |
| `distance/distance` | `sensor_msgs/msg/Range` |

Each camera image topic has a corresponding `camera_info` topic. The bridge's control timer republishes IMU, GPS, and the other scalar sensors at 100 Hz by default; their measurement timestamps indicate when the simulator produced new samples. Camera and LiDAR request intervals are configurable through the launch arguments shown above.

### Blocks UAV with sensors

The same packaged Blocks environment supports the SimpleFlight multirotor. `ros2/settings/blocks_uav_sensors.json` selects it automatically and enables the same camera and sensor streams as the car profile. The cameras sit 0.5 metres forward of the drone centre, the LiDAR sits 0.8 metres above it to clear the propellers, and its vertical field of view extends down to -45 degrees to see the ground in flight.

Stop the existing simulator and bridge, then run these commands in separate terminals from `HERCULES/ros2`.

Start the UAV simulator:

```shell
bash ../docker/Blocks_packaged_Linux_52_32/Linux/Blocks.sh \
  -settings="$(pwd)/settings/blocks_uav_sensors.json" \
  -windowed -ResX=1280 -ResY=720
```

Start the bridge with API control enabled. This enables API control and arms the simulated UAV:

```shell
source install/setup.bash &&
  ros2 launch airsim_ros_pkgs airsim_node.launch.py \
    host_ip:=127.0.0.1 enable_api_control:=True \
    image_update_interval:=0.1 lidar_update_interval:=0.1 \
    enable_object_transforms_list:=False
```

Open the UAV view:

```shell
source install/setup.bash &&
  rviz2 -d rviz2_configs/blocks_uav.rviz
```

The view follows `SimpleFlight/odom_local`, with sensor topics below `/airsim_node/SimpleFlight/`. Use the same ROS domain and middleware settings in each terminal. Once the bridge has initialized, take off from another sourced terminal:

```shell
ros2 service call /airsim_node/SimpleFlight/takeoff \
  airsim_interfaces/srv/Takeoff "{wait_on_last_task: false}"
```

The drone climbs and holds position. With `wait_on_last_task: false`, the service's `success` field reports command submission and returns before flight completes. With `true`, it waits for the takeoff and reports whether it completed. The packaged Cosys-AirSim Blocks demo reports `success: false` even when the drone has climbed and holds position, because its AirLib treats a move that stops within the drone's distance accuracy of the target as unfinished; simulators built from HERCULES report it correctly. With the demo, use the asynchronous command above and check the odometry in RViz to confirm flight. Land with:

```shell
ros2 service call /airsim_node/SimpleFlight/land \
  airsim_interfaces/srv/Land "{wait_on_last_task: true}"
```

To return to the car, stop the UAV simulator and bridge and use the car settings and RViz commands above.

#### Keyboard flight

With the UAV airborne and the bridge running, use terminal keyboard controls:

```shell
source install/setup.bash &&
  ros2 run airsim_ros_pkgs uav_keyboard_teleop.py
```

Keep that terminal focused while flying. Controls follow the drone's heading:

| Keys | Action |
| --- | --- |
| W / S | Forward / backward |
| A / D | Left / right |
| R / F | Up / down |
| Q / E | Turn left / right |
| Space | Stop and hover |
| G | Land and exit |
| X or Ctrl-C | Exit and leave the drone hovering |

Hold a movement key to keep moving. Commands stop within 0.6 seconds of releasing the key. The defaults are 0.8 m/s horizontally, 0.5 m/s vertically, and 30 degrees/s turning; adjust them with `--speed`, `--vertical-speed`, and `--yaw-rate`. This controller uses the existing ROS bridge and needs no additional Python packages. Use the same ROS domain and middleware settings as the bridge and RViz. To fly a drone of a Hero-mode team through `hercules_node`, add `--bridge hercules`; see [Running a UAV–UGV Team](hero_team_quickstart.md#fly-the-drones-from-ros-2).

If the keyboard controller reports no active UAV bridge, check the bridge terminal for errors. The simulator window can remain open after the ROS bridge has exited. Restart the bridge using the command above and wait for `AirsimROSWrapper Initialized!` before starting the controller again.

## HERCULES bridge (`hercules_node`)

`airsim_node` is the generic Cosys-AirSim bridge. HERCULES simulations, including the heterogeneous Hero mode, are bridged by `hercules_node` ([`airsim_node_hercules.cpp`](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_ros_pkgs/src/airsim_node_hercules.cpp), wrapper in [`hercules_ros_wrapper.cpp`](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_ros_pkgs/src/hercules_ros_wrapper.cpp)), which the same workspace builds. It reads the `airsim_node` [parameters](#parameters) plus `topic_namespace`, the prefix of its topics and services (default: the node's name), and `update_imu_every_n_sec`, the IMU publishing period (default 0.01 s).

Per-vehicle topics are below `<topic_namespace>/<Vehicle>/` and follow the `airsim_node` layout described below, with these differences: odometry is on `ground_truth/odom_local`, there is no `car_state` or `environment` topic, `car_cmd` exists only with `enable_api_control:=True`, and `instance_segmentation_refresh` and `object_transforms_refresh` are per-vehicle services.

### Hero team launch

In Hero mode the simulator serves multirotors on RPC port 41451 and cars on 41452, and each bridge connects to one port. Start one bridge per port with:

```shell
source install/setup.bash &&
  ros2 launch airsim_ros_pkgs hercules_hero_team.launch.py
```

[`hercules_hero_team.launch.py`](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_ros_pkgs/launch/hercules_hero_team.launch.py) starts `hercules_uav_bridge` on 41451, which also publishes `/clock`, and `hercules_ugv_bridge` on 41452. Each bridge serves the vehicles of its kind by `VehicleType`: `SimpleFlight`, `PX4Multirotor`, `ArduCopter` and `ArduCopterSolo` on the multirotor bridge, all other types on the car bridge. Both use the prefix `/hercules_node`, so each vehicle's topics are `/hercules_node/<Vehicle>/...`; the car bridge's copies of the world-level `origin_geo_point`, `instance_segmentation_labels`, `object_transforms` and `reset` move to `/hercules_ugv_bridge/...`. Launch arguments are `host_ip` (default `localhost`), `enable_api_control` (`False`), `enable_object_transforms_list` (`True`), `is_vulkan` (`True`) and `output` (`screen`). [Running a UAV–UGV Team](hero_team_quickstart.md#4-start-the-ros-2-bridges) walks through a complete example.

To run a single bridge, [`airsim_node_hercules.launch.py`](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_ros_pkgs/launch/airsim_node_hercules.launch.py) starts one `hercules_node` with `host_port` defaulting to 41452.

### Frames

`world` and `odom_local` below are the defaults of the `world_frame_id` and `odom_frame_id` parameters. Static transforms are published once on `/tf_static`; poses are converted from AirSim's NED convention by negating Y and Z.

```text
world
└── <Vehicle>                                static: spawn pose from the vehicle's settings
    └── <Vehicle>/ground_truth/odom_local    odometry (TF and nav_msgs/Odometry)
        ├── <Vehicle>/<camera>_body          static: camera pose from settings
        ├── <Vehicle>/<camera>_optical       static: images and camera_info
        ├── <Vehicle>/<lidar>                static: lidar point clouds
        └── <Vehicle>/<distance sensor>      static: ranges
```

- Odometry has header frame `<Vehicle>` and child frame `<Vehicle>/ground_truth/odom_local`, and is relative to the vehicle's pose at the bridge's first update, which is its spawn pose when the bridge starts before the vehicle moves.
- Camera images and `camera_info` are stamped `<Vehicle>/<camera>_optical` with the image capture time; `camera_info` is published with every image.
- Lidar clouds are stamped `<Vehicle>/<lidar>`; GPU lidar and echo clouds use `<Vehicle>/<sensor>` the same way. Cameras, lidars, GPU lidars and echo sensors marked `External` hang off `world` instead.
- Distance sensors publish `sensor_msgs/Range` stamped `<Vehicle>/<sensor>`, with a static transform from the odometry frame.
- IMU messages are stamped `<Vehicle>/ground_truth/odom_local`; GPS, magnetometer and barometer messages are stamped `<Vehicle>`.
- For a vehicle imported from a URDF, the sensors listed in its `Urdf` `SensorFrames` setting get no static transform from the bridge; `robot_state_publisher` publishes them from the robot description. See [Importing a URDF Robot](urdf_import.md#ros-2-frames).

## Using HERCULES ROS wrapper

The ROS wrapper is composed of two ROS nodes - the first is a wrapper over HERCULES's multirotor C++ client library, and the second is a simple PD position controller.
Let's look at the ROS API for both nodes:

### HERCULES ROS Wrapper Node

#### Publishers:
The publishers will be automatically created based on the settings in the `settings.json` file for all vehicles and the sensors.

- `/airsim_node/VEHICLE-NAME/car_state` [airsim_interfaces::CarState](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/CarState.msg)
  The state of the car if the vehicle is of this sim-mode type.

- `/airsim_node/VEHICLE-NAME/computervision_state` [airsim_interfaces::ComputerVisionState](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/ComputerVisionState.msg)
  The state of the computer vision actor if the vehicle is of this sim-mode type.

- `/airsim_node/origin_geo_point` [airsim_interfaces::GPSYaw](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/GPSYaw.msg)
  GPS coordinates corresponding to global frame. This is set in the HERCULES [settings.json](settings.md) file under the `OriginGeopoint` key.

- `/airsim_node/VEHICLE-NAME/global_gps` [sensor_msgs::NavSatFix](https://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html)
  This the current GPS coordinates of the drone in HERCULES.

- `/airsim_node/VEHICLE-NAME/environment` [airsim_interfaces::Environment](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/Environment.msg)

- `/airsim_node/VEHICLE-NAME/odom_local` [nav_msgs::Odometry](https://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
  Odometry frame (default name: odom_local, launch name and frame type are configurable) wrt take-off point.

- `/airsim_node/VEHICLE-NAME/CAMERA-NAME_IMAGE-TYPE/camera_info` [sensor_msgs::CameraInfo](https://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)
  Optionally if the image type is annotation the annotation layer name is also included in the topic name.

- `/airsim_node/VEHICLE-NAME/CAMERA-NAME_IMAGE-TYPE/image` [sensor_msgs::Image](https://docs.ros.org/api/sensor_msgs/html/msg/Image.html)
  RGB or float image depending on image type requested in settings.json. Optionally if the image type is annotation the annotation layer name is also included in the topic name.

- `/tf` [tf2_msgs::TFMessage](https://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html)

- `/airsim_node/VEHICLE-NAME/altimeter/SENSOR_NAME` [airsim_interfaces::Altimeter](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/Altimeter.msg)
  This the current altimeter reading for altitude, pressure, and [QNH](https://en.wikipedia.org/wiki/QNH)

- `/airsim_node/VEHICLE-NAME/imu/SENSOR_NAME` [sensor_msgs::Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
  IMU sensor data.

- `/airsim_node/VEHICLE-NAME/magnetometer/SENSOR_NAME` [sensor_msgs::MagneticField](http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html)
  Measurement of magnetic field vector/compass.

- `/airsim_node/VEHICLE-NAME/distance/SENSOR_NAME` [sensor_msgs::Range](http://docs.ros.org/api/sensor_msgs/html/msg/Range.html)
  Measurement of distance from an active ranger, such as infrared or IR

- `/airsim_node/VEHICLE-NAME/lidar/points/SENSOR_NAME/` [sensor_msgs::PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)
  LIDAR pointcloud 

- `/airsim_node/VEHICLE-NAME/lidar/labels/SENSOR_NAME/` [airsim_interfaces::StringArray](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/StringArray.msg)
  Custom message type with an array of string that are the labels for each point in the pointcloud of the lidar sensor

- `/airsim_node/VEHICLE-NAME/gpulidar/points/SENSOR_NAME/` [sensor_msgs::PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)
  GPU LIDAR pointcloud. The instance segmentation/annotation color data is stored in the rgb field of the pointcloud. The intensity data is stored as well in the intensity field

- `/airsim_node/VEHICLE-NAME/echo/active/points/SENSOR_NAME/` [sensor_msgs::PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)
  Echo sensor pointcloud for active sensing

- `/airsim_node/VEHICLE-NAME/echo/passive/points/SENSOR_NAME/` [sensor_msgs::PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)
  Echo sensor pointcloud for passive sensing

- `/airsim_node/VEHICLE-NAME/echo/active/labels/SENSOR_NAME/` [airsim_interfaces::StringArray](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/StringArray.msg)
  Custom message type with an array of string that are the labels for each point in the pointcloud for the active echo pointcloud

- `/airsim_node/VEHICLE-NAME/echo/passive/labels/SENSOR_NAME/` [airsim_interfaces::StringArray](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/StringArray.msg)
  Custom message type with an array of string that are the labels for each point in the pointcloud for the passive echo pointcloud

- `/airsim_node/instance_segmentation_labels` [airsim_interfaces::InstanceSegmentationList](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/InstanceSegmentationList.msg)
  Custom message type with an array of a custom messages that are the names, color and index of the instance segmentation system for each object in the world.
   
- `/airsim_node/object_transforms` [airsim_interfaces::ObjectTransformsList](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/ObjectTransformsList.msg)
  Custom message type with an array of [geometry_msgs::TransformStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html) that are the transforms of all objects in the world, each child frame ID is the object name.
   
#### Subscribers:

- `/airsim_node/VEHICLE-NAME/vel_cmd_body_frame` [airsim_interfaces::VelCmd](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/VelCmd.msg)
  
- `/airsim_node/VEHICLE-NAME/vel_cmd_world_frame` [airsim_interfaces::VelCmd](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/VelCmd.msg)
  
- `/airsim_node/all_robots/vel_cmd_body_frame` [airsim_interfaces::VelCmd](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/VelCmd.msg)
  Set velocity command for all drones.

- `/airsim_node/all_robots/vel_cmd_world_frame` [airsim_interfaces::VelCmd](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/VelCmd.msg)

- `/airsim_node/group_of_robots/vel_cmd_body_frame` [airsim_interfaces::VelCmdGroup](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/VelCmdGroup.msg)
  Set velocity command for a specific set of drones.
- 
- `/airsim_node/group_of_robots/vel_cmd_world_frame` [airsim_interfaces::VelCmdGroup](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/VelCmdGroup.msg)
  Set velocity command for a specific set of drones.

- `/gimbal_angle_euler_cmd` [airsim_interfaces::GimbalAngleEulerCmd](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/GimbalAngleEulerCmd.msg)
  Gimbal set point in euler angles.

- `/gimbal_angle_quat_cmd` [airsim_interfaces::GimbalAngleQuatCmd](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/GimbalAngleQuatCmd.msg)
  Gimbal set point in quaternion.

- `/airsim_node/VEHICLE-NAME/car_cmd` [airsim_interfaces::CarControls](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/CarControls.msg)
Throttle, brake, steering and gear selections for control. Both automatic and manual transmission control possible, see the [`car_joy`](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_ros_pkgs/scripts/car_joy) script for an example (a ROS 1 `rospy` script that has not been ported to ROS 2 and is not installed by the package).

#### Services:

- `/airsim_node/VEHICLE-NAME/land` [airsim_interfaces::Land](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/srv/Land.srv)

- `/airsim_node/VEHICLE-NAME/takeoff` [airsim_interfaces::Takeoff](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/srv/Takeoff.srv)

- `/airsim_node/all_robots/land` [airsim_interfaces::Land](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/srv/Land.srv)
 land all drones

- `/airsim_node/all_robots/takeoff` [airsim_interfaces::Takeoff](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/srv/Takeoff.srv)
 take-off all drones

- `/airsim_node/group_of_robots/land` [airsim_interfaces::LandGroup](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/srv/LandGroup.srv)
 land a specific set of drones

- `/airsim_node/group_of_robots/takeoff` [airsim_interfaces::TakeoffGroup](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/srv/TakeoffGroup.srv)
 take-off a specific set of drones

- `/airsim_node/reset` [airsim_interfaces::Reset](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/srv/Reset.srv)
 Resets *all* vehicles

- `/airsim_node/instance_segmentation_refresh` [airsim_interfaces::RefreshInstanceSegmentation](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/srv/RefreshInstanceSegmentation.srv)
 Refresh the instance segmentation list

- `/airsim_node/object_transforms_refresh` [airsim_interfaces::RefreshObjectTransforms](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/srv/RefreshObjectTransforms.srv)
 Refresh the object transforms list

  

#### Parameters:

- `/airsim_node/host_ip` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch.py`
  Default: localhost
  The IP of the machine running the HERCULES RPC API server.

- `/airsim_node/host_port` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch.py`
  Default: 41451
  The port of the machine running the HERCULES RPC API server.

- `/airsim_node/enable_api_control` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch.py`
  Default: false
  Set the API control and arm the drones on startup. If not set to true no control is available. 

- `/airsim_node/enable_object_transforms_list` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch.py`
  Default: true
  Retrieve the object transforms list from the HERCULES API at the start or with the service to refresh. If disabled this is not available but can save time on startup.

- `/airsim_node/is_vulkan` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch.py`
  Default: True
  If using Vulkan, the image encoding is switched from rgb8 to bgr8. 

- `/airsim_node/world_frame_id` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch.py`
  Default: world

- `/airsim_node/odom_frame_id` [string]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch.py`
  Default: odom_local

- `/airsim_node/update_airsim_control_every_n_sec` [double]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch.py`
  Default: 0.01 seconds.
  Timer callback frequency for updating drone odom and state from HERCULES, and sending in control commands.
  The current RPClib interface to unreal engine maxes out at 50 Hz.
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter.

- `/airsim_node/update_airsim_img_response_every_n_sec` [double]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch.py`
  Default: 0.01 seconds.
  Timer callback frequency for receiving images from all cameras in HERCULES.
  The speed will depend on number of images requested and their resolution.
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter.

- `/airsim_node/update_lidar_every_n_sec` [double]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch.py`
  Default: 0.01 seconds.
  Timer callback frequency for receiving images from all Lidar data in HERCULES.
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter.


- `/airsim_node/update_gpulidar_every_n_sec` [double]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch.py`
  Default: 0.01 seconds.
  Timer callback frequency for receiving images from all GPU-Lidar data in HERCULES.
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter.

- `/airsim_node/update_echo_every_n_sec` [double]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch.py`
  Default: 0.01 seconds.
  Timer callback frequency for receiving images from all echo sensor data in HERCULES.
  Timer callbacks in ROS run at maximum rate possible, so it's best to not touch this parameter.

- `/airsim_node/publish_clock` [double]
  Set in: `$(airsim_ros_pkgs)/launch/airsim_node.launch.py`
  Default: false
  Will publish the ros /clock topic if set to true.

### Simple PID Position Controller Node

#### Parameters:

- PD controller parameters:
  * `/pd_position_node/kp_x` [double],
    `/pd_position_node/kp_y` [double],
    `/pd_position_node/kp_z` [double],
    `/pd_position_node/kp_yaw` [double]
    Proportional gains

  * `/pd_position_node/kd_x` [double],
    `/pd_position_node/kd_y` [double],
    `/pd_position_node/kd_z` [double],
    `/pd_position_node/kd_yaw` [double]
    Derivative gains

  * `/pd_position_node/reached_thresh_xyz` [double]
    Threshold euler distance (meters) from current position to setpoint position

  * `/pd_position_node/reached_yaw_degrees` [double]
    Threshold yaw distance (degrees) from current position to setpoint position

- `/pd_position_node/update_control_every_n_sec` [double]
  Default: 0.01 seconds

#### Services:

- `/airsim_node/VEHICLE-NAME/gps_goal` [Request: [airsim_interfaces::SetGPSPosition](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/srv/SetGPSPosition.srv)]
  Target gps position + yaw.
  In **absolute** altitude.

- `/airsim_node/VEHICLE-NAME/local_position_goal` [Request: [airsim_interfaces::SetLocalPosition](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/srv/SetLocalPosition.srv)]
  Target local position + yaw in global frame.

#### Subscribers:

- `/airsim_node/origin_geo_point` [airsim_interfaces::GPSYaw](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/GPSYaw.msg)
  Listens to home geo coordinates published by `airsim_node`.

- `/airsim_node/VEHICLE-NAME/odom_local` [nav_msgs::Odometry](https://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
  Listens to odometry published by `airsim_node`

#### Publishers:

- `/vel_cmd_world_frame` [airsim_interfaces::VelCmd](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/VelCmd.msg)
  Sends velocity command to `airsim_node`

- `/vel_cmd_body_frame` [airsim_interfaces::VelCmd](https://github.com/lunarlab-gatech/HERCULES/blob/main/ros2/src/airsim_interfaces/msg/VelCmd.msg)
  Sends velocity command to `airsim_node`

#### Global params

- Dynamic constraints. These can be changed in `dynamic_constraints.launch.py`:
    * `/max_vel_horz_abs` [double]
  Maximum horizontal velocity of the drone (meters/second)

    * `/max_vel_vert_abs` [double]
  Maximum vertical velocity of the drone (meters/second)

    * `/max_yaw_rate_degree` [double]
  Maximum yaw rate (degrees/second)