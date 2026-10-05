#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('octree_file',
                              description='octomap (.bt) file to serve, e.g. ~/maps/site.bt'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join('~', 'rviz_config', 'octomap.rviz'),
                              description='RViz config with an OctoMap display (optional)'),
        OpaqueFunction(function=_nodes),
    ])


def _nodes(context):
    octree_file = os.path.expanduser(LaunchConfiguration('octree_file').perform(context))
    if not os.path.isfile(octree_file):
        raise RuntimeError('octree_file %s does not exist' % octree_file)

    # Launch the octomap_server node.
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{'octree_file': octree_file}]
    )

    rviz_config_file = os.path.expanduser(LaunchConfiguration('rviz_config').perform(context))

    # Launch rviz2 (if the config file exists, pass it; otherwise, launch without a config).
    if os.path.exists(rviz_config_file):
        rviz2_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    else:
        rviz2_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )

    return [octomap_server_node, rviz2_node]
