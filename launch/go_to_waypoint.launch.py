import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = "mav_trajectory_generation_ros2"
    
    default_yaml_path = os.path.join(get_package_share_directory(pkg_name), 'config', 'go_to_waypoint.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_yaml_path,
            description='Path to the config file.'
        ),
        DeclareLaunchArgument(
            'waypoint_node_namespace',
            default_value='',
            description='waypoint_node_namespace'
        ),

        Node(
            package=pkg_name,
            executable='go_to_waypoint_node',
            name='go_to_waypoint_node',
            namespace=LaunchConfiguration('waypoint_node_namespace'),
            parameters=[LaunchConfiguration('config_file')],
            remappings=[
                ('odom', '/mavros/local_position/odom'),
                ('path_segments', 'path_segments'),
                ('waypoint_navigator_polynomial_markers', 'waypoint_navigator_polynomial_markers'),
                ('waypoint', 'waypoint'),
                ('command/trajectory', 'geometric_controller/multi_dof_setpoint')
            ]
        ),
    ])
