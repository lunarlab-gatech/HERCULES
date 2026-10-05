#!/usr/bin/env python3

import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion
import yaml
import cv2
import os
import math
import numpy as np

class MapPublisher(Node):
    def __init__(self):
        super().__init__('custom_map_server')
        
        # Declare parameters
        self.declare_parameter('yaml_file', '/home/sgarimella34/multi-robot-coordination/trajectory_data/occupancy_grid_maps/Ausenv_ground_OGM_0p5m.yaml')
        self.declare_parameter('continuous_publish', True)
        self.declare_parameter('ogm_topic', 'Ausenv_0mAlt_OGM_0p5m')  # Topic name parameter
        self.declare_parameter('altitude', 0.0)  # New parameter for the OGM altitude

        yaml_file = self.get_parameter('yaml_file').value
        self.continuous_publish = self.get_parameter('continuous_publish').value
        topic_name = self.get_parameter('ogm_topic').value  # Retrieve topic name parameter
        altitude = self.get_parameter('altitude').value     # Retrieve altitude parameter

        # Load the YAML file
        try:
            with open(yaml_file, 'r') as f:
                map_yaml = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML file: {yaml_file} ({e})")
            return

        # Extract parameters from the YAML
        image_file = map_yaml.get('image')
        resolution = map_yaml.get('resolution')
        origin = map_yaml.get('origin')  # Expecting [x, y, theta]
        negate = map_yaml.get('negate')
        occupied_thresh = map_yaml.get('occupied_thresh')
        free_thresh = map_yaml.get('free_thresh')

        if image_file is None or resolution is None or origin is None:
            self.get_logger().error("YAML file is missing required parameters.")
            return

        # Build the full image path (assuming image is in the same directory as the YAML)
        yaml_dir = os.path.dirname(os.path.abspath(yaml_file))
        image_path = os.path.join(yaml_dir, image_file)

        # Load the map image as grayscale
        map_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if map_image is None:
            self.get_logger().error(f"Failed to load map image from {image_path}")
            return

        # Create and fill the OccupancyGrid message
        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.header.frame_id = 'map'
        self.occupancy_grid.info.resolution = resolution
        self.occupancy_grid.info.width = map_image.shape[1]
        self.occupancy_grid.info.height = map_image.shape[0]
        self.occupancy_grid.info.origin.position.x = float(origin[0])
        self.occupancy_grid.info.origin.position.y = float(origin[1])
        self.occupancy_grid.info.origin.position.z = float(altitude)  # Use altitude parameter here

        # Convert yaw (origin[2]) to a quaternion for the map's orientation
        yaw = float(origin[2])
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        self.occupancy_grid.info.origin.orientation = q

        # Convert the image to occupancy data.
        rows, cols = map_image.shape
        data = np.empty((rows, cols), dtype=np.int8)

        for y in range(rows):
            for x in range(cols):
                occ = map_image[y, x] / 255.0  # Normalize to [0,1]
                
                # If NEGATE in yaml, do this
                if negate:
                    occ = 1.0 - occ

                if occ > occupied_thresh:
                    if (occ >= 0.88):
                        value = -1
                    else:
                        value = 50
                elif occ < free_thresh:
                    value = 100
                else:
                    value = 100
                
                
                data[y, x] = value

        # Flip the data vertically (ROS occupancy grids have origin at bottom-left).
        data = np.flipud(data)
        self.occupancy_grid.data = data.flatten().tolist()

        # Create a publisher on the topic specified by the parameter "ogm_topic"
        self.publisher = self.create_publisher(OccupancyGrid, topic_name, 10)

        if self.continuous_publish:
            # Publish continuously every second
            self.timer = self.create_timer(1.0, self.publish_map)
        else:
            # Publish only once and shutdown
            self.publish_map()
            self.get_logger().info("Map published once. Exiting...")
            rclpy.shutdown()

    def publish_map(self):
        self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.occupancy_grid)
        self.get_logger().info(f"Publishing occupancy grid map on {self.publisher.topic_name}", once=True)

def main(args=None):
    def interrupt_once(signum, frame):
        # Under ros2 launch, Ctrl-C reaches this process from the terminal and from launch;
        # only the first one may interrupt, so the cleanup below completes.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt_once)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MapPublisher()

    if not node.continuous_publish:
        # If publishing once, exit before entering spin loop
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    # If continuous, keep spinning
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
