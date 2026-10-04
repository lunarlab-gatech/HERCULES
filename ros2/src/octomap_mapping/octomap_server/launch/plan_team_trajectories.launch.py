"""Plan coverage or leader-follower trajectories for a HERCULES UAV-UGV team.

Publishes the two occupancy grid maps written by PythonClient/hero/make_planning_maps.py
and runs trajectory_planner_OGM_node on them. The planner reads every Drone*/Husky* vehicle
and its start pose from the settings file, plans the UGVs on the ground map and the drones
on the drone map, and writes <output_folder>/<Vehicle>_trajectory.txt relative to each
vehicle's start, ready for the waypoint run scripts. Everything stops once the
trajectories are saved; exit_when_done:=false keeps the nodes running until Ctrl-C, for
example to look at the maps in RViz.

  ros2 launch octomap_server plan_team_trajectories.launch.py \\
      settings_file:=$PWD/settings/hero_blocks_team.json \\
      ground_map:=$PWD/../trajectory_data/maps/blocks_ground.yaml \\
      drone_map:=$PWD/../trajectory_data/maps/blocks_drone_10m.yaml \\
      output_folder:=$PWD/../trajectory_data flight_pattern:=Convoy
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

GROUND_TOPIC = 'planning/ground_map'
DRONE_TOPIC = 'planning/drone_map'


def generate_launch_description():
    args = [
        DeclareLaunchArgument('settings_file', description='settings.json with the Drone*/Husky* team'),
        DeclareLaunchArgument('ground_map', description='<name>_ground.yaml from make_planning_maps.py'),
        DeclareLaunchArgument('drone_map', description='<name>_drone_<alt>m.yaml from make_planning_maps.py'),
        DeclareLaunchArgument('output_folder', description='folder for <Vehicle>_trajectory.txt'),
        DeclareLaunchArgument('flight_pattern', default_value='RandomExplore',
                              description='drones without a FlightPattern: RandomExplore (coverage) or '
                                          'Convoy (follow the nearest UGV)'),
        DeclareLaunchArgument('drone_altitude', default_value='10.0',
                              description='metres above the PlayerStart; use the drone map\'s altitude'),
        DeclareLaunchArgument('square_size', default_value='100.0',
                              description='side of the planning area in metres; the map --size'),
        DeclareLaunchArgument('trajectory_length', default_value='100.0',
                              description='metres per vehicle unless its settings give TrajectoryLength'),
        DeclareLaunchArgument('exit_when_done', default_value='true',
                              description='stop once every trajectory is saved'),
    ]

    def number(name):
        # `square_size:=60` would otherwise reach the node as an integer and be rejected.
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    def map_publisher(name, yaml_file, topic, altitude):
        return Node(
            package='octomap_server', executable='publish_saved_ogm.py', name=name, output='screen',
            parameters=[{'yaml_file': yaml_file, 'ogm_topic': topic, 'altitude': altitude,
                         'continuous_publish': True}])

    planner = Node(
        package='octomap_server', executable='trajectory_planner_OGM_node',
        name='trajectory_planner_OGM', output='screen',
        parameters=[{
            'settings_file': LaunchConfiguration('settings_file'),
            # The planner appends "<Vehicle>_trajectory.txt" directly.
            'output_folder': [LaunchConfiguration('output_folder'), '/'],
            'ground_map_topic': GROUND_TOPIC,
            'drone_map_topic': DRONE_TOPIC,
            'default_flight_pattern': LaunchConfiguration('flight_pattern'),
            'drone_altitude': number('drone_altitude'),
            'square_size': number('square_size'),
            'trajectory_length': number('trajectory_length'),
            'trajectories_relative_to_start': True,
            # The default horizon (50 segments) would cut trajectories short of trajectory_length.
            'planning_horizon_steps': 0,
            'exit_when_done': ParameterValue(LaunchConfiguration('exit_when_done'), value_type=bool),
        }])

    return LaunchDescription(args + [
        map_publisher('ground_map_publisher', LaunchConfiguration('ground_map'), GROUND_TOPIC, 0.0),
        map_publisher('drone_map_publisher', LaunchConfiguration('drone_map'), DRONE_TOPIC,
                      number('drone_altitude')),
        planner,
        # The map publishers only serve the planner.
        RegisterEventHandler(OnProcessExit(target_action=planner,
                                           on_exit=[Shutdown(reason='planner finished')])),
    ])
