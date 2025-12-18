from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    share_player = FindPackageShare('bunker02_dataset_player').find('bunker02_dataset_player')
    share_odom = FindPackageShare('radar_simple_odometry').find('radar_simple_odometry')
    share_viz = FindPackageShare('bunker02_rviz').find('bunker02_rviz')

    player_launch = os.path.join(share_player, 'launch', 'play_radar.launch.py')
    odom_launch = os.path.join(share_odom, 'launch', 'radar_odom.launch.py')
    rviz_cfg = os.path.join(share_viz, 'rviz', 'radar_odom.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(player_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(odom_launch)),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', rviz_cfg]),
    ])
