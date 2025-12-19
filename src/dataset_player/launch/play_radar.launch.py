from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('bunker02_dataset_player').find('bunker02_dataset_player')
    default_cfg = os.path.join(pkg_share, 'config', 'player.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_cfg),
        Node(
            package='dataset_player',
            executable='radar_player',
            name='radar_player',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        )
    ])
