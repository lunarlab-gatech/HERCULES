"""
Publish the TF tree of every URDF-imported vehicle in a HERCULES settings file.

For each vehicle whose settings carry ``Urdf.RobotDescription`` this starts a
robot_state_publisher in the vehicle's namespace with ``frame_prefix`` set to
``<vehicle>/``. The imported URDF is rooted at ``ground_truth/<odom_frame>``,
so its frames hang below the ``<vehicle>/ground_truth/odom_local`` frame the
HERCULES ROS 2 wrapper broadcasts from the simulator's odometry, and the
wrapper leaves the sensor frames of such vehicles to this publisher.

    ros2 launch hercules_urdf_import robot_description.launch.py \
        settings:=$HOME/Documents/AirSim/settings.json
"""

import json
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def urdf_vehicles(settings_path):
    """(vehicle name, URDF path) for every vehicle imported from a URDF."""
    with open(settings_path, 'r', encoding='utf-8') as stream:
        settings = json.load(stream)
    base = os.path.dirname(os.path.abspath(settings_path))
    found = []
    for name, vehicle in sorted(settings.get('Vehicles', {}).items()):
        description = (vehicle.get('Urdf') or {}).get('RobotDescription')
        if description:
            found.append((name, os.path.join(base, os.path.expanduser(description))))
    return found


def _publishers(context):
    settings_path = os.path.expanduser(LaunchConfiguration('settings').perform(context))
    wanted = [v for v in LaunchConfiguration('vehicles').perform(context).split(',') if v]
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    vehicles = urdf_vehicles(settings_path)
    if wanted:
        missing = sorted(set(wanted) - {name for name, _ in vehicles})
        if missing:
            raise RuntimeError('no URDF vehicle named %s in %s'
                               % (', '.join(missing), settings_path))
        vehicles = [(name, path) for name, path in vehicles if name in wanted]
    if not vehicles:
        return [LogInfo(msg='No vehicle in %s has Urdf.RobotDescription; nothing to '
                            'publish.' % settings_path)]
    actions = []
    for name, path in vehicles:
        with open(path, 'r', encoding='utf-8') as stream:
            description = stream.read()
        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=name,
            output='screen',
            parameters=[{
                'robot_description': description,
                'frame_prefix': name + '/',
                'use_sim_time': use_sim_time,
            }],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'settings',
            default_value=os.path.join('~', 'Documents', 'AirSim', 'settings.json'),
            description='HERCULES settings.json holding the imported vehicles'),
        DeclareLaunchArgument(
            'vehicles', default_value='',
            description='comma-separated vehicle names (default: every URDF vehicle)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='use /clock (set when the wrapper runs with publish_clock)'),
        OpaqueFunction(function=_publishers),
    ])
