"""ROS 2 bridges for a HERCULES Hero-mode team (drones and Huskies in one simulation).

Hero mode serves drones on RPC port 41451 and ground vehicles on 41452, and hercules_node
bridges one port. This starts one bridge per port under distinct node names that share the
/hercules_node topic prefix, so each vehicle's topics stay /hercules_node/<Vehicle>/...
Only the drone bridge publishes /clock.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TOPIC_NAMESPACE = '/hercules_node'

# Both bridges see the same world, so the UGV bridge's copies of the world-level topics and
# the reset service move under its own name instead of duplicating the drone bridge's.
WORLD_LEVEL_NAMES = ('origin_geo_point', 'instance_segmentation_labels', 'object_transforms', 'reset')


def bridge(name, host_port, publish_clock, remappings=()):
    return Node(
        package='airsim_ros_pkgs',
        executable='hercules_node',
        name=name,
        output=LaunchConfiguration('output'),
        parameters=[{
            'is_vulkan': LaunchConfiguration('is_vulkan'),
            'update_airsim_img_response_every_n_sec': 0.5,
            'update_airsim_control_every_n_sec': 0.05,
            'update_lidar_every_n_sec': 0.5,
            'update_gpulidar_every_n_sec': 0.01,
            'update_echo_every_n_sec': 0.05,
            'use_sim_time': True,
            'publish_clock': publish_clock,
            'host_ip': LaunchConfiguration('host_ip'),
            'host_port': host_port,
            'enable_api_control': LaunchConfiguration('enable_api_control'),
            'enable_object_transforms_list': LaunchConfiguration('enable_object_transforms_list'),
            'topic_namespace': TOPIC_NAMESPACE,
        }],
        remappings=list(remappings))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output', default_value='screen'),
        DeclareLaunchArgument('host_ip', default_value='localhost'),
        DeclareLaunchArgument('is_vulkan', default_value='True'),
        DeclareLaunchArgument('enable_api_control', default_value='False'),
        DeclareLaunchArgument('enable_object_transforms_list', default_value='True'),
        bridge('hercules_uav_bridge', 41451, True),
        bridge('hercules_ugv_bridge', 41452, False, [
            (f'{TOPIC_NAMESPACE}/{n}', f'/hercules_ugv_bridge/{n}') for n in WORLD_LEVEL_NAMES]),
    ])
