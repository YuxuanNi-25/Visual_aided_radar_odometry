from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('radar_odometry').find('radar_odometry')
    default_cfg = os.path.join(pkg_share, 'config', 'odom.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_cfg),
        Node(
            package='radar_odometry',
            executable='radar_odometry',
            name='radar_odometry',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        )
    ])
