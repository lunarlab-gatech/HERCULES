HERCULES DATA COLLECTOR

This repository contains scripts and documentation for collecting
perfectly synchronized simulation data from HERCULES
(running in Unreal Engine 5.2.1) running in ROS 2 Humble. You can
record IMU, odometry, synchronized RGB, depth & segmentation images,
and LiDAR point clouds, and pack them into a ROS 2 bag for playback
or algorithm development.

DATA LAYOUT

All raw data is saved under a single root directory
(e.g. /media/.../raw_data_hercules/):

raw_data_hercules/
├── imu.txt          # one line per IMU sample @200 Hz
├── odom.txt         # one line per odometry sample @odom_hz
├── rgb/             # scene (RGB) images (.png)
├── depth/           # depth-planar images (.png)
├── seg/             # instance segmentation images (.png)
└── lidar/           # LiDAR point clouds (.npy)

imu.txt
Each line has 7 space-separated fields:
    t ax ay az gx gy gz
  • t        — sim time in seconds (relative to t=0 at first sample),
               with six decimal places
  • ax,ay,az — linear acceleration (m/s²) in NED (North/East/Down)
  • gx,gy,gz — angular velocity (rad/s) in NED
Maps to ROS 2’s sensor_msgs/msg/Imu.

odom.txt
Each line has 8 space-separated fields:
    t px py pz qw qx qy qz
  • t         — same sim-time base as IMU
  • px,py,pz  — position in meters (NED)
  • qw,qx,qy,qz — orientation quaternion (world→body, NED)
Maps to ROS 2’s nav_msgs/msg/Odometry.

IMAGES
Files in rgb/, depth/, seg/ are named t.png (six-decimal sim time).  
Maps to sensor_msgs/msg/Image on topics:
  • /camera/color/image_raw ← rgb/t.png  
  • /camera/depth/image_raw ← depth/t.png  
  • /camera/seg/image_raw   ← seg/t.png
Headers: stamp = t, frame_id = "camera_link"

LIDAR
Files in lidar/ are NumPy .npy arrays of shape (N×3), float32,
points in NED. Filename is t.npy.
Maps to sensor_msgs/msg/PointCloud2.

RECORDING SCRIPT

Use hercules_data_collector.py:
  cd ~/multi-robot-coordination/HERCULES/PythonClient/hero
  chmod +x hercules_data_collector.py
  python3 hercules_data_collector.py \
    --duration 30 \
    --vehicle Husky1 \
    --camera front_center \
    --outdir /media/.../raw_data_hercules

Options:
  --duration  (s)   Total sim time to record  
  --vehicle        Vehicle name in settings.json  
  --camera         Camera name  
  --outdir         Root output folder  

PACKING INTO A ROS 2 BAG

Use pack_to_ros2bag.py:
  chmod +x pack_to_ros2bag.py
  ./pack_to_ros2bag.py \
    /media/.../raw_data_hercules \
    ~/bags/hercules_sim

This generates a rosbag2 SQLite3 database with topics:
  /imu     → sensor_msgs/Imu  
  /odom    → nav_msgs/Odometry  
  /camera/color/image_raw  
  /camera/depth/image_raw  
  /camera/seg/image_raw  
  /lidar   → sensor_msgs/PointCloud2  
  /tf      → tf2_msgs/TFMessage

All messages share the exact sim-time stamps recorded,
so playback at any rate preserves perfect synchronization.

COORDINATE FRAMES

• NED (North/East/Down) – used by AirSim and all recorded data.  
• odom frame – inertial world origin.  
• base_link frame – vehicle’s body-fixed frame.  
• camera_link frame – camera optical origin, +X forward, +Y right, +Z down.

Transform tree:
  odom
    └ base_link (identity)
        └ camera_link
