from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Resolve parameter file relative to the package share to avoid
    # issues when the launch is executed from another working directory.
    params_file = LaunchConfiguration(
        'slam_params_file',
        default=PathJoinSubstitution([
            FindPackageShare('bringup'), 'config', 'mapper_params_online_async.yaml'
        ])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=params_file,
            description='Full path to the slam_toolbox parameters file'
        ),

        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file,
                        {'use_sim_time': use_sim_time}],
        ),
    ])
