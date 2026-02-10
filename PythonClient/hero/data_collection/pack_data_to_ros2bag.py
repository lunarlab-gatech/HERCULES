#!/usr/bin/env python3
"""
pack_to_ros2bag.py

Reads raw_data_dir/{<imu-file>, <odom-files>, <rgb-folder>/, depth/, seg/, lidar/}
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
  python3 pack_data_to_ros2bag.py \
    /media/…/raw_data_hercules/test1_1husky \
    /media/…/converted_ros2bags/test6_1husky \
    --exclude-topics depth,seg,lidar \
    --rgb-folder rgb_gray \
    --imu-file synthetic_imu.txt \
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

def make_pointcloud2(points, stamp, frame_id="Husky1/LidarSensor1"):
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

    # 1) Load IMU
    imu_path = os.path.join(raw_data_dir, imu_file)
    imu_data = parse_txt(imu_path)

    # 2) Load any number of ODOM files
    odom_data_map = {}
    for odom_fname in odom_files:
        odom_key = os.path.splitext(os.path.basename(odom_fname))[0]
        path = os.path.join(raw_data_dir, odom_fname)
        odom_data_map[odom_key] = parse_txt(path)

    # 3) Images
    img_topics = {
        "rgb":   "camera/color/image_raw",
        "depth": "camera/depth/image_raw",
        "seg":   "camera/seg/image_raw",
    }
    img_map = {}
    for sub in img_topics:
        folder = rgb_folder if sub=="rgb" else sub
        img_map[sub] = {
            float(os.path.basename(f)[:-4]): f
            for f in glob.glob(f"{raw_data_dir}/{folder}/*.png")
        }

    # 4) LiDAR
    lidar_map = {
        float(os.path.basename(f)[:-4]): f
        for f in glob.glob(f"{raw_data_dir}/lidar/*.npy")
    }

    # 5) Gather all timestamps
    all_ts = set(imu_data)
    for od in odom_data_map.values():
        all_ts |= set(od)
    all_ts |= set(img_map["rgb"]) | set(lidar_map)
    all_ts = sorted(all_ts)

    # clear out old bag
    if os.path.exists(bag_out):
        shutil.rmtree(bag_out)

    # open writer
    writer = SequentialWriter()
    storage_opts = StorageOptions(uri=bag_out, storage_id='sqlite3')
    conv_opts = ConverterOptions(input_serialization_format='cdr',
                                 output_serialization_format='cdr')
    writer.open(storage_opts, conv_opts)

    # helper
    def want(key):
        return ("all" in include or key in include) and key not in exclude

    # register IMU
    if want("imu"):
        writer.create_topic(TopicMetadata(
            name="/imu",
            type="sensor_msgs/msg/Imu",
            serialization_format="cdr"
        ))

    # register each odom
    for odom_key in odom_data_map:
        if want(odom_key):
            writer.create_topic(TopicMetadata(
                name=f"/{odom_key}",
                type="nav_msgs/msg/Odometry",
                serialization_format="cdr"
            ))

    # register images
    for sub, topic in img_topics.items():
        if want(sub):
            writer.create_topic(TopicMetadata(
                name=f"/{topic}",
                type="sensor_msgs/msg/Image",
                serialization_format="cdr"
            ))

    # register lidar & tf
    if want("lidar"):
        writer.create_topic(TopicMetadata(
            name="/lidar",
            type="sensor_msgs/msg/PointCloud2",
            serialization_format="cdr"
        ))
    if want("tf"):
        writer.create_topic(TopicMetadata(
            name="/tf",
            type="tf2_msgs/msg/TFMessage",
            serialization_format="cdr"
        ))

    # write
    for t in all_ts:
        ns = int(t * 1e9)
        stamp = Time(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000)

        # IMU
        if want("imu") and t in imu_data:
            ax, ay, az, gx, gy, gz = imu_data[t]
            m = Imu()
            m.header.stamp = stamp
            m.header.frame_id = "Husky1/ground_truth/odom_local"
            m.linear_acceleration.x = ax
            m.linear_acceleration.y = ay
            m.linear_acceleration.z = az
            m.angular_velocity.x    = gx
            m.angular_velocity.y    = gy
            m.angular_velocity.z    = gz
            writer.write("/imu", serialize_message(m), ns)

        # each odom
        for odom_key, odom_data in odom_data_map.items():
            if want(odom_key) and t in odom_data:
                px, py, pz, qw, qx, qy, qz = odom_data[t]
                m = Odometry()
                m.header.stamp = stamp
                m.header.frame_id = "Husky1"
                m.child_frame_id = "Husky1/ground_truth/odom_local"
                m.pose.pose.position.x = px
                m.pose.pose.position.y = py
                m.pose.pose.position.z = pz
                m.pose.pose.orientation.w = qw
                m.pose.pose.orientation.x = qx
                m.pose.pose.orientation.y = qy
                m.pose.pose.orientation.z = qz
                writer.write(f"/{odom_key}", serialize_message(m), ns)

        # images
        for sub, topic in img_topics.items():
            if want(sub) and t in img_map[sub]:
                path = img_map[sub][t]
                if sub=="rgb":
                    cvim = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2RGB)
                    enc = "rgb8"
                else:
                    cvim = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
                    enc = "mono8"
                im = bridge.cv2_to_imgmsg(cvim, encoding=enc)
                im.header.stamp = stamp
                im.header.frame_id = "Husky1/front_center_optical"
                writer.write(f"/{topic}", serialize_message(im), ns)

        # lidar
        if want("lidar") and t in lidar_map:
            pts = np.load(lidar_map[t])
            pc2 = make_pointcloud2(pts, t, frame_id="Husky1/LidarSensor1")
            writer.write("/lidar", serialize_message(pc2), ns)

        # tf (publish each odom as transform)
        if want("tf"):
            for odom_key, odom_data in odom_data_map.items():
                if t in odom_data:
                    px, py, pz, qw, qx, qy, qz = odom_data[t]
                    tf = TransformStamped()
                    tf.header.stamp = stamp
                    tf.header.frame_id = "Husky1"
                    tf.child_frame_id = "Husky1/ground_truth/odom_local"
                    tf.transform.translation.x = px
                    tf.transform.translation.y = py
                    tf.transform.translation.z = pz
                    tf.transform.rotation.w = qw
                    tf.transform.rotation.x = qx
                    tf.transform.rotation.y = qy
                    tf.transform.rotation.z = qz
                    writer.write("/tf",
                                 serialize_message(TFMessage(transforms=[tf])),
                                 ns)

    print(f"Written bag to {bag_out}")

if __name__=="__main__":
    p = argparse.ArgumentParser(
        description="Pack raw HERCULES data into a ROS2 bag"
    )
    p.add_argument("raw_data_dir",
                   help="Folder containing imu.txt, odom.txt, etc.")
    p.add_argument("bag_out",
                   help="Empty (or non-existent) folder to create the bag in")
    p.add_argument("--include-topics", default="all",
                   help="Comma-separated list: imu,odom,odom_secondary,rgb,depth,seg,lidar,tf or 'all'")
    p.add_argument("--exclude-topics", default="",
                   help="Comma-separated list of streams to skip")
    p.add_argument("--rgb-folder", default="rgb",
                   help="Subfolder under raw_data_dir to load RGB images from")
    p.add_argument("--imu-file", default="imu.txt",
                   help="Name of the IMU text file under raw_data_dir")
    p.add_argument("--odom-files", default="odom.txt",
                   help="Comma-separated list of odometry text files to load")
    args = p.parse_args()

    include = set(args.include_topics.split(",")) if args.include_topics else {"all"}
    exclude = set(args.exclude_topics.split(",")) if args.exclude_topics else set()
    odom_files = args.odom_files.split(",")

    main(args.raw_data_dir,
         args.bag_out,
         include,
         exclude,
         args.rgb_folder,
         args.imu_file,
         odom_files)
