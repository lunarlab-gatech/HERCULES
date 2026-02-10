#!/usr/bin/env python3
"""
pack_to_ros2bag.py

Reads raw_data_dir/{<vehicle>/imu.txt, <vehicle>/odom-files, <vehicle>/rgb, depth, seg, lidar}
and writes a ROS2 Humble bag at bag_out, deleting any existing bag_out first.

You can filter which streams go into the bag via:
  --include-topics imu,odom,odom_secondary,rgb,depth,seg,lidar,tf   (default: all)
  --exclude-topics imu,tf                                           (default: none)

You can override which subfolder holds RGB images:
  --rgb-folder <folder_name>                        (default: rgb)

You can override which IMU txt file to load:
  --imu-file <file_name>                            (default: imu.txt)

You can specify one or more odometry files (comma-separated):
  --odom-files odom.txt,odom_secondary.txt          (default: odom.txt)

Example:
  python3 pack_to_ros2bag.py \
    /media/.../raw_data_hercules/test2_2uav2ugv \
    /media/.../converted_ros2bags/test2_2uav2ugv \
    --exclude-topics depth,seg,lidar \
    --rgb-folder rgb_gray \
    --imu-file imu.txt \
    --odom-files odom.txt,odom_groundtruth.txt
"""

import os
import shutil
import glob
import argparse

import numpy as np
import cv2

from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu, Image, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
from rclpy.serialization import serialize_message

# Image topics mapping
IMG_TOPICS = {
    "rgb":   "camera/color/image_raw",
    "depth": "camera/depth/image_raw",
    "seg":   "camera/seg/image_raw",
}

# Default sensor names
CAMERA_NAME = "front_center"
LIDAR_NAME  = "LidarSensor1"


def parse_txt(fname):
    """Return dict mapping timestamp → list of float fields."""
    d = {}
    with open(fname, 'r') as f:
        for line in f:
            parts = line.strip().split()
            t = float(parts[0])
            vals = list(map(float, parts[1:]))
            d[t] = vals
    return d


def make_pointcloud2(points, stamp, frame_id):
    """Convert Nx3 ndarray → sensor_msgs/PointCloud2."""
    msg = PointCloud2()
    msg.header.stamp = Time(sec=int(stamp), nanosec=int((stamp % 1) * 1e9))
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = points.shape[0]
    msg.is_dense = True
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = points.astype(np.float32).tobytes()
    return msg


def main(raw_data_dir, bag_out, include, exclude, rgb_folder, imu_file, odom_files):
    bridge = CvBridge()

    # Determine which streams to include
    include = set(include)
    exclude = set(exclude)
    def want(key):
        return ("all" in include or key in include) and key not in exclude

    # Detect vehicle subdirectories
    vehicles = sorted(
        [d for d in os.listdir(raw_data_dir)
         if os.path.isdir(os.path.join(raw_data_dir, d))]
    )

    # Load data per vehicle
    imu_data_map   = {}
    odom_data_map  = {}
    img_map        = {}
    lidar_map      = {}

    for veh in vehicles:
        root = os.path.join(raw_data_dir, veh)
        # IMU
        imu_path = os.path.join(root, imu_file)
        imu_data_map[veh] = parse_txt(imu_path) if want("imu") else {}

        # Odometries
        odom_data_map[veh] = {}
        for fname in odom_files:
            key = os.path.splitext(os.path.basename(fname))[0]
            path = os.path.join(root, fname)
            odom_data_map[veh][key] = parse_txt(path)

        # Images
        img_map[veh] = {}
        for sub in IMG_TOPICS:
            folder = rgb_folder if sub=="rgb" else sub
            folder_path = os.path.join(root, folder)
            img_map[veh][sub] = {
                float(os.path.basename(f)[:-4]): f
                for f in glob.glob(os.path.join(folder_path, "*.png"))
            }

        # LiDAR
        lidar_dir = os.path.join(root, "lidar")
        lidar_map[veh] = {
            float(os.path.basename(f)[:-4]): f
            for f in glob.glob(os.path.join(lidar_dir, "*.npy"))
        }

    # Build global timestamp set
    all_ts = set()
    for veh in vehicles:
        if want("imu"):    all_ts |= set(imu_data_map[veh])
        for od in odom_data_map[veh].values(): all_ts |= set(od)
        for sub in IMG_TOPICS:                   all_ts |= set(img_map[veh][sub])
        if want("lidar"): all_ts |= set(lidar_map[veh])
    all_ts = sorted(all_ts)

    # Remove existing bag
    if os.path.exists(bag_out): shutil.rmtree(bag_out)

    # Open writer
    writer = SequentialWriter()
    storage_opts = StorageOptions(uri=bag_out, storage_id='sqlite3')
    conv_opts = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    writer.open(storage_opts, conv_opts)

    # Register topics per vehicle
    # IMU
    if want("imu"):
        for veh in vehicles:
            writer.create_topic(TopicMetadata(
                name=f"/{veh}/imu",
                type="sensor_msgs/msg/Imu",
                serialization_format="cdr"
            ))
    # Odometry
    for veh in vehicles:
        for key in odom_data_map[veh]:
            if want(key):
                writer.create_topic(TopicMetadata(
                    name=f"/{veh}/{key}",
                    type="nav_msgs/msg/Odometry",
                    serialization_format="cdr"
                ))
    # Images
    for veh in vehicles:
        for sub, topic in IMG_TOPICS.items():
            if want(sub):
                writer.create_topic(TopicMetadata(
                    name=f"/{veh}/{topic}",
                    type="sensor_msgs/msg/Image",
                    serialization_format="cdr"
                ))
    # LiDAR
    if want("lidar"):
        for veh in vehicles:
            writer.create_topic(TopicMetadata(
                name=f"/{veh}/lidar",
                type="sensor_msgs/msg/PointCloud2",
                serialization_format="cdr"
            ))
    # TF
    if want("tf"):
        for veh in vehicles:
            writer.create_topic(TopicMetadata(
                name=f"/{veh}/tf",
                type="tf2_msgs/msg/TFMessage",
                serialization_format="cdr"
            ))

    # Write messages
    for t in all_ts:
        ns = int(t * 1e9)
        stamp = Time(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000)

        for veh in vehicles:
            # IMU
            if want("imu") and t in imu_data_map[veh]:
                ax, ay, az, gx, gy, gz = imu_data_map[veh][t]
                m = Imu()
                m.header.stamp = stamp
                m.header.frame_id = f"{veh}/ground_truth/odom_local"
                m.linear_acceleration.x = ax
                m.linear_acceleration.y = ay
                m.linear_acceleration.z = az
                m.angular_velocity.x    = gx
                m.angular_velocity.y    = gy
                m.angular_velocity.z    = gz
                writer.write(f"/{veh}/imu", serialize_message(m), ns)

            # Odometry
            for key, odom_data in odom_data_map[veh].items():
                if want(key) and t in odom_data:
                    px, py, pz, qw, qx, qy, qz = odom_data[t]
                    m = Odometry()
                    m.header.stamp = stamp
                    m.header.frame_id = veh
                    m.child_frame_id = f"{veh}/ground_truth/odom_local"
                    m.pose.pose.position.x = px
                    m.pose.pose.position.y = py
                    m.pose.pose.position.z = pz
                    m.pose.pose.orientation.w = qw
                    m.pose.pose.orientation.x = qx
                    m.pose.pose.orientation.y = qy
                    m.pose.pose.orientation.z = qz
                    writer.write(f"/{veh}/{key}", serialize_message(m), ns)

            # Images
            for sub, topic in IMG_TOPICS.items():
                if want(sub) and t in img_map[veh][sub]:
                    path = img_map[veh][sub][t]
                    if sub == "rgb":
                        cvim = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2RGB)
                        encoding = "rgb8"
                    else:
                        cvim = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
                        encoding = "mono8"
                    im = bridge.cv2_to_imgmsg(cvim, encoding=encoding)
                    im.header.stamp = stamp
                    im.header.frame_id = f"{veh}/{CAMERA_NAME}_optical"
                    writer.write(f"/{veh}/{topic}", serialize_message(im), ns)

            # LiDAR
            if want("lidar") and t in lidar_map[veh]:
                pts = np.load(lidar_map[veh][t])
                pc2 = make_pointcloud2(pts, t, frame_id=f"{veh}/{LIDAR_NAME}")
                writer.write(f"/{veh}/lidar", serialize_message(pc2), ns)

            # TF
            if want("tf"):
                for key, odom_data in odom_data_map[veh].items():
                    if t in odom_data:
                        px, py, pz, qw, qx, qy, qz = odom_data[t]
                        tf = TransformStamped()
                        tf.header.stamp = stamp
                        tf.header.frame_id = veh
                        tf.child_frame_id = f"{veh}/ground_truth/odom_local"
                        tf.transform.translation.x = px
                        tf.transform.translation.y = py
                        tf.transform.translation.z = pz
                        tf.transform.rotation.w = qw
                        tf.transform.rotation.x = qx
                        tf.transform.rotation.y = qy
                        tf.transform.rotation.z = qz
                        writer.write(f"/{veh}/tf",
                                     serialize_message(TFMessage(transforms=[tf])),
                                     ns)

    print(f"Written bag to {bag_out}")


if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description="Pack raw HERCULES data into a ROS2 bag"
    )
    p.add_argument("raw_data_dir",
                   help="Folder containing per-vehicle imu.txt, odom.txt, and sensor subfolders")
    p.add_argument("bag_out",
                   help="Folder to create the ROS2 bag in")
    p.add_argument("--include-topics", default="all",
                   help="Comma-separated: imu,odom,odom_secondary,rgb,depth,seg,lidar,tf or 'all'")
    p.add_argument("--exclude-topics", default="",
                   help="Comma-separated list of streams to skip")
    p.add_argument("--rgb-folder", default="rgb",
                   help="Subfolder under each vehicle dir to load RGB images from")
    p.add_argument("--imu-file", default="imu.txt",
                   help="IMU text file name under each vehicle dir")
    p.add_argument("--odom-files", default="odom.txt",
                   help="Comma-separated list of odometry text files to load from each vehicle dir")
    args = p.parse_args()

    include = args.include_topics.split(",") if args.include_topics else ["all"]
    exclude = args.exclude_topics.split(",") if args.exclude_topics else []
    odom_files = args.odom_files.split(",")

    main(args.raw_data_dir,
         args.bag_out,
         include,
         exclude,
         args.rgb_folder,
         args.imu_file,
         odom_files)
