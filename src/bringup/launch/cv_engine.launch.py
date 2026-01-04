"""Launch file for the `cv_engine` node.

Provides launch arguments to set `use_sim_time` and the image topic to subscribe to.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    image_topic = LaunchConfiguration('image_topic')
    start_realsense = LaunchConfiguration('start_realsense')
    enable_rgbd = LaunchConfiguration('enable_rgbd')
    enable_sync = LaunchConfiguration('enable_sync')
    align_depth_enable = LaunchConfiguration('align_depth_enable')
    enable_color = LaunchConfiguration('enable_color')
    enable_depth = LaunchConfiguration('enable_depth')

    # Path to the realsense rs_launch.py
    rs_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock'),
        DeclareLaunchArgument('image_topic', default_value='/camera/camera/color/image_raw', description='Image topic to subscribe to'),
        DeclareLaunchArgument('start_realsense', default_value='true', description='Automatically start RealSense launch'),
        DeclareLaunchArgument('enable_rgbd', default_value='true', description='Enable RGBD in RealSense launch'),
        DeclareLaunchArgument('enable_sync', default_value='true', description='Enable synced streams'),
        DeclareLaunchArgument('align_depth_enable', default_value='true', description='Enable depth alignment to color'),
        DeclareLaunchArgument('enable_color', default_value='true', description='Enable color stream'),
        DeclareLaunchArgument('enable_depth', default_value='true', description='Enable depth stream'),

        Node(
            package='cv_engine',
            executable='cv_engine',
            name='cv_engine',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/camera/camera/color/image_raw', image_topic)],
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(rs_launch_path),
        #     launch_arguments={
        #         'enable_rgbd': enable_rgbd,
        #         'enable_sync': enable_sync,
        #         'align_depth.enable': align_depth_enable,
        #         'enable_color': enable_color,
        #         'enable_depth': enable_depth,
        #     }.items(),
        #     condition=IfCondition(start_realsense),
        # ),
    ])
