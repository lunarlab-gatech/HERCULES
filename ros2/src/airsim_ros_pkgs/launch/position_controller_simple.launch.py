import launch
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def constraint(name, default):
    # declared by dynamic_constraints.launch.py when that is included first
    return ParameterValue(LaunchConfiguration(name, default=default), value_type=float)


def generate_launch_description():
    ld = launch.LaunchDescription([
            Node(
                package='airsim_ros_pkgs',
                executable='pd_position_controller_simple_node',
                name='pid_position_node',
                output='screen',
                parameters=[{
                    'update_control_every_n_sec': 0.01,
                    
                    'kp_x': 0.30,
                    'kp_y': 0.30,
                    'kp_z': 0.30,
                    'kp_yaw': 0.30,
                    
                    'kd_x': 0.05,
                    'kd_y': 0.05,
                    'kd_z': 0.05,
                    'kd_yaw': 0.05,
                   
                    'reached_thresh_xyz': 0.1,
                    'reached_yaw_degrees': 5.0,

                    'max_vel_horz_abs': constraint('max_vel_horz_abs', '0.5'),
                    'max_vel_vert_abs': constraint('max_vel_vert_abs', '10.0'),
                    'max_yaw_rate_degree': constraint('max_yaw_rate_degree', '1.0'),
                }
            ]
        )
    ])

    return ld
