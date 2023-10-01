import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share_path = get_package_share_directory('mav_trajectory_generation_ros2')
    default_yaml_path = pkg_share_path + '/config/trjectory_sampler.yaml'
    
    config = LaunchConfiguration('config_file', default=default_yaml_path)

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config,
            description='Full path to the YAML config file to be used'
        ),
        DeclareLaunchArgument(
            'sampler_namespace',
            default_value='',
            description='sampler_node_namespace'
        ),
        
        LogInfo(msg=[
            "Loading config file: ", config
        ]),

        Node(
            package='mav_trajectory_generation_ros2',
            executable='trajectory_sampler_node',
            name='trajectory_sampler_node',
            namespace=LaunchConfiguration('sampler_namespace'),
            output='screen',
            parameters=[config],
            remappings=[
                ('path_segments', 'path_segments'),
                ('path_segments_4D', 'path_segments_4D'),
                ('command/trajectory', 'geometric_controller/multi_dof_setpoint'),
            ]
        ),
    ])
