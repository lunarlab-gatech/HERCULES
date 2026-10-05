"""
Run the Hero team bridges against a fake simulator.

fake_airsim_server answers the simulator's RPCs from a settings file, so
this exercises hercules_node end to end without Unreal Engine: which
vehicles each bridge serves, the frames its messages are stamped with, the
TF tree (including a vehicle imported from a URDF, whose sensor frames come
from robot_state_publisher), camera_info/image pairing, odometry and reset.
"""

import json
import math
import os
import tempfile
import threading
import time
import unittest

from airsim_interfaces.srv import Reset
from ament_index_python.packages import get_package_share_directory
from hercules_urdf_import import import_robot, ImportOptions
import launch
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image, Imu, Range
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

NS = '/hercules_node'

ROVER_URDF = """<?xml version="1.0"?>
<robot name="rover">
  <link name="base_link">
    <visual><geometry><box size="0.8 0.5 0.3"/></geometry></visual>
  </link>
  <link name="camera_link"/>
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/><child link="camera_link"/>
    <origin xyz="0.4 0 0.3" rpy="0 0.2 0"/>
  </joint>
  <link name="lidar_link"/>
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/><child link="lidar_link"/>
    <origin xyz="0 0 0.5"/>
  </joint>
  <gazebo reference="camera_link">
    <sensor type="camera" name="front_cam">
      <camera><horizontal_fov>1.5707963</horizontal_fov>
        <image><width>8</width><height>6</height></image></camera>
    </sensor>
  </gazebo>
  <gazebo reference="lidar_link">
    <sensor type="ray" name="rover_lidar">
      <update_rate>10</update_rate>
      <ray><scan>
        <horizontal><samples>360</samples><min_angle>-3.14159265</min_angle>
          <max_angle>3.14159265</max_angle></horizontal>
        <vertical><samples>16</samples><min_angle>-0.26</min_angle><max_angle>0.26</max_angle></vertical>
      </scan><range><min>0.2</min><max>50</max></range></ray>
    </sensor>
  </gazebo>
</robot>
"""

DRONE_SETTINGS = {
    'SettingsVersion': 2.0,
    'SimMode': 'Hero',
    'Vehicles': {
        'Drone1': {
            'VehicleType': 'SimpleFlight',
            'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'Roll': 0.0, 'Pitch': 0.0, 'Yaw': 0.0,
            'Cameras': {
                'front_center': {
                    'X': 0.5, 'Y': 0.0, 'Z': -0.1, 'Roll': 0.0, 'Pitch': -10.0, 'Yaw': 0.0,
                    'CaptureSettings': [
                        {'ImageType': 0, 'Width': 8, 'Height': 6, 'FOV_Degrees': 90},
                        {'ImageType': 1, 'Width': 8, 'Height': 6, 'FOV_Degrees': 90},
                    ],
                },
            },
            'Sensors': {
                'imu': {'SensorType': 2, 'Enabled': True},
                'range_down': {
                    'SensorType': 5, 'Enabled': True, 'MinDistance': 0.1, 'MaxDistance': 20,
                    'X': 0.0, 'Y': 0.0, 'Z': 0.1, 'Roll': 0.0, 'Pitch': -90.0, 'Yaw': 0.0,
                },
            },
        },
    },
}


@pytest.mark.launch_test
def generate_test_description():
    work = tempfile.mkdtemp(prefix='hercules_bridge_test_')
    base = os.path.join(work, 'base.json')
    with open(base, 'w') as stream:
        json.dump(DRONE_SETTINGS, stream)
    urdf = os.path.join(work, 'rover.urdf')
    with open(urdf, 'w') as stream:
        stream.write(ROVER_URDF)
    # a custom-named UGV, spawned 2 m north and 3 m east facing east
    result = import_robot(ImportOptions(urdf, 'Rover7', 'skid', work, merge_settings=base,
                                        position=(2.0, 3.0, 0.0), yaw_deg=90.0))
    settings = result.files['settings']

    server = ExecuteProcess(
        cmd=[os.environ['FAKE_AIRSIM_SERVER'], '--settings', settings,
             '--multirotor-port', '41451', '--car-port', '41452'],
        output='screen')
    bridges = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('airsim_ros_pkgs'), 'launch',
            'hercules_hero_team.launch.py')),
        launch_arguments={'is_vulkan': 'False'}.items())
    descriptions = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hercules_urdf_import'), 'launch',
            'robot_description.launch.py')),
        launch_arguments={'settings': settings}.items())
    return launch.LaunchDescription([
        server, bridges, descriptions, launch_testing.actions.ReadyToTest(),
    ]), {'imported': result}


class Collector:
    """Subscribe to everything the checks need and keep the latest messages."""

    def __init__(self):
        self.node = rclpy.create_node('hercules_bridge_checker')
        self.lock = threading.Lock()
        self.messages = {}
        self.static_tf = []
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        latched = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(TFMessage, '/tf_static', self._on_static, latched)
        for topic, kind in (
                (NS + '/Drone1/front_center_Scene/image', Image),
                (NS + '/Drone1/front_center_Scene/camera_info', CameraInfo),
                (NS + '/Drone1/front_center_DepthPlanar/image', Image),
                (NS + '/Drone1/front_center_DepthPlanar/camera_info', CameraInfo),
                (NS + '/Drone1/imu/imu', Imu),
                (NS + '/Drone1/distance/range_down', Range),
                (NS + '/Drone1/ground_truth/odom_local', Odometry),
                (NS + '/Rover7/ground_truth/odom_local', Odometry),
                (NS + '/Rover7/front_cam_Scene/camera_info', CameraInfo)):
            self.node.create_subscription(kind, topic, self._keeper(topic), 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _keeper(self, topic):
        def keep(msg):
            with self.lock:
                self.messages.setdefault(topic, []).append(msg)
        return keep

    def _on_static(self, msg):
        with self.lock:
            self.static_tf.extend(msg.transforms)

    def wait_for(self, topic, count=1, timeout=60.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                found = list(self.messages.get(topic, []))
            if len(found) >= count:
                return found
            time.sleep(0.1)
        raise AssertionError('no message on %s within %.0f s' % (topic, timeout))

    def lookup(self, parent, child, timeout=60.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.buffer.can_transform(parent, child, rclpy.time.Time()):
                return self.buffer.lookup_transform(parent, child, rclpy.time.Time()).transform
            time.sleep(0.1)
        raise AssertionError('no transform %s -> %s within %.0f s' % (parent, child, timeout))

    def static_parents(self, child):
        with self.lock:
            return {t.header.frame_id for t in self.static_tf if t.child_frame_id == child}

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()


def rotate(q, v):
    """Rotate vector v by quaternion q (geometry_msgs Quaternion)."""
    w, x, y, z = q.w, q.x, q.y, q.z
    vx, vy, vz = v
    cx, cy, cz = y * vz - z * vy + w * vx, z * vx - x * vz + w * vy, x * vy - y * vx + w * vz
    return (vx + 2 * (y * cz - z * cy), vy + 2 * (z * cx - x * cz), vz + 2 * (x * cy - y * cx))


class TestHeroBridges(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.c = Collector()

    @classmethod
    def tearDownClass(cls):
        cls.c.close()
        rclpy.shutdown()

    def test_each_bridge_serves_its_vehicle_family_by_type(self):
        # Rover7 is not named Husky*, yet the UGV bridge serves it as a car
        self.c.wait_for(NS + '/Rover7/ground_truth/odom_local')
        self.c.wait_for(NS + '/Drone1/ground_truth/odom_local')
        rover = self.c.node.get_publishers_info_by_topic(NS + '/Rover7/ground_truth/odom_local')
        drone = self.c.node.get_publishers_info_by_topic(NS + '/Drone1/ground_truth/odom_local')
        self.assertEqual([p.node_name for p in rover], ['hercules_ugv_bridge'])
        self.assertEqual([p.node_name for p in drone], ['hercules_uav_bridge'])

    def test_camera_info_pairs_with_images(self):
        for image_type in ('Scene', 'DepthPlanar'):
            prefix = NS + '/Drone1/front_center_' + image_type
            images = self.c.wait_for(prefix + '/image', count=2)
            infos = self.c.wait_for(prefix + '/camera_info', count=2)
            # every image goes out with a camera_info of the same stamp
            image_stamps = {(m.header.stamp.sec, m.header.stamp.nanosec) for m in images}
            info_stamps = {(m.header.stamp.sec, m.header.stamp.nanosec) for m in infos}
            self.assertTrue(image_stamps & info_stamps, 'no camera_info carries an image stamp')
            for msg in images + infos:
                self.assertEqual(msg.header.frame_id, 'Drone1/front_center_optical')
            self.assertEqual((infos[-1].width, infos[-1].height), (8, 6))
            self.assertAlmostEqual(infos[-1].k[0], 4.0, places=4)  # f = (w/2) / tan(45 deg)

    def test_sensor_frames(self):
        imu = self.c.wait_for(NS + '/Drone1/imu/imu')[-1]
        self.assertEqual(imu.header.frame_id, 'Drone1/ground_truth/odom_local')
        rng = self.c.wait_for(NS + '/Drone1/distance/range_down')[-1]
        self.assertEqual(rng.header.frame_id, 'Drone1/range_down')
        # pitched -90 deg in settings: the range axis (x) points down in ROS
        down = self.c.lookup('Drone1/ground_truth/odom_local', 'Drone1/range_down')
        axis = rotate(down.rotation, (1.0, 0.0, 0.0))
        for got, want in zip(axis, (0.0, 0.0, -1.0)):
            self.assertAlmostEqual(got, want, places=4)
        self.assertAlmostEqual(down.translation.z, -0.1, places=4)

    def test_wrapper_camera_tf_for_plain_vehicles(self):
        self.c.lookup('Drone1/ground_truth/odom_local', 'Drone1/front_center_optical')
        self.assertEqual(self.c.static_parents('Drone1/front_center_body'),
                         {'Drone1/ground_truth/odom_local'})

    def test_urdf_vehicle_frames_come_from_robot_state_publisher(self, imported):
        camera = imported.vehicle['Cameras']['front_cam']
        tf = self.c.lookup('Rover7/ground_truth/odom_local', 'Rover7/front_cam_optical')
        self.assertAlmostEqual(tf.translation.x, camera['X'], places=4)
        self.assertAlmostEqual(tf.translation.z, -camera['Z'], places=4)
        self.c.lookup('Rover7/ground_truth/odom_local', 'Rover7/rover_lidar')
        # only robot_state_publisher, hanging them off the URDF links
        self.assertEqual(self.c.static_parents('Rover7/front_cam_body'), {'Rover7/camera_link'})
        self.assertEqual(self.c.static_parents('Rover7/rover_lidar'), {'Rover7/lidar_link'})
        info = self.c.wait_for(NS + '/Rover7/front_cam_Scene/camera_info')[-1]
        self.assertEqual(info.header.frame_id, 'Rover7/front_cam_optical')

    def test_odometry_is_relative_to_the_start_pose(self):
        # Rover7 starts facing east and drives forward: in its start frame
        # that is +x, whatever its world heading.
        time.sleep(2.0)
        odom = self.c.wait_for(NS + '/Rover7/ground_truth/odom_local')[-1]
        p = odom.pose.pose.position
        self.assertGreater(p.x, 0.5)
        self.assertLess(abs(p.y), 0.05 * p.x + 1e-3)
        self.assertLess(abs(p.z), 1e-3)
        q = odom.pose.pose.orientation
        self.assertAlmostEqual(abs(q.w), 1.0, places=4)
        # world -> Rover7 places the start pose: 2 m north, 3 m east, facing east
        start = self.c.lookup('world', 'Rover7')
        self.assertAlmostEqual(start.translation.x, 2.0, places=4)
        self.assertAlmostEqual(start.translation.y, -3.0, places=4)
        forward = rotate(start.rotation, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(forward[1], -1.0, places=4)  # east is -y in the wrapper's world

    def test_reset_reports_success(self):
        client = self.c.node.create_client(Reset, NS + '/reset')
        self.assertTrue(client.wait_for_service(timeout_sec=60.0))
        future = client.call_async(Reset.Request())
        deadline = time.monotonic() + 30.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.1)
        self.assertTrue(future.done())
        self.assertTrue(future.result().success)

    def test_no_nan_in_static_transforms(self):
        self.c.lookup('Drone1/ground_truth/odom_local', 'Drone1/front_center_optical')
        with self.c.lock:
            transforms = list(self.c.static_tf)
        for t in transforms:
            values = (t.transform.translation.x, t.transform.translation.y,
                      t.transform.translation.z, t.transform.rotation.w)
            self.assertTrue(all(math.isfinite(v) for v in values), t.child_frame_id)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
