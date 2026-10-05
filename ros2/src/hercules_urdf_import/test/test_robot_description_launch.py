"""
Check robot_state_publisher on an imported URDF.

It must accept the URDF and publish the frames the HERCULES ROS 2 wrapper
stamps its data with, at the poses written to the settings.
"""

import math
import os
import sys
import tempfile
import time
import unittest

import pytest

# Needs a sourced ROS 2 installation; the rest of the suite does not.
launch = pytest.importorskip('launch')
launch_testing = pytest.importorskip('launch_testing')
rclpy = pytest.importorskip('rclpy')

from launch.actions import IncludeLaunchDescription  # noqa: E402, I100
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402
import launch_testing.actions  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from hercules_urdf_import import import_robot, ImportOptions  # noqa: E402, I100
from hercules_urdf_import.transforms import (  # noqa: E402
    OPTICAL_FROM_BODY, quat_from_rpy, Transform)

URDF = os.path.join(HERE, 'fixtures', 'test_robot_description', 'urdf', 'skid_robot.urdf')
LAUNCH = os.path.join(os.path.dirname(HERE), 'launch', 'robot_description.launch.py')


@pytest.mark.launch_test
def generate_test_description():
    output = tempfile.mkdtemp(prefix='hercules_urdf_import_')
    result = import_robot(ImportOptions(URDF, 'Husky3', 'skid', output))
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(LAUNCH),
            launch_arguments={'settings': result.files['settings']}.items()),
        launch_testing.actions.ReadyToTest(),
    ]), {'result': result}


def wrapper_tf(entry):
    w, x, y, z = quat_from_rpy(*(math.radians(entry[k]) for k in ('Roll', 'Pitch', 'Yaw')))
    return Transform((w, x, -y, -z), (entry['X'], -entry['Y'], -entry['Z']))


def to_transform(msg):
    t, r = msg.transform.translation, msg.transform.rotation
    return Transform((r.w, r.x, r.y, r.z), (t.x, t.y, t.z))


class TestImportedTfTree(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('hercules_urdf_import_tf_check')
        cls.buffer = Buffer()
        cls.listener = TransformListener(cls.buffer, cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def lookup(self, parent, child, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.buffer.can_transform(parent, child, rclpy.time.Time()):
                return to_transform(self.buffer.lookup_transform(
                    parent, child, rclpy.time.Time()))
        self.fail('no transform %s -> %s within %.0f s' % (parent, child, timeout))

    def test_sensor_frames_match_settings(self, result):
        body = 'Husky3/ground_truth/odom_local'
        camera = result.vehicle['Cameras']['front_cam']
        self.assertTrue(self.lookup(body, 'Husky3/front_cam_body')
                        .is_close(wrapper_tf(camera), 1e-5))
        self.assertTrue(self.lookup(body, 'Husky3/front_cam_optical')
                        .is_close(wrapper_tf(camera) * OPTICAL_FROM_BODY, 1e-5))
        for name in ('os1_64', 'sonar_front'):
            self.assertTrue(self.lookup(body, 'Husky3/' + name)
                            .is_close(wrapper_tf(result.vehicle['Sensors'][name]), 1e-5))

    def test_robot_links_are_published(self):
        wheel = self.lookup('Husky3/base_footprint', 'Husky3/front_left_wheel')
        self.assertAlmostEqual(wheel.translation[0], 0.256, places=6)
        self.assertAlmostEqual(wheel.translation[2], 0.13 + 0.03282, places=6)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15])
