from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import os

def launch_setup(context, *args, **kwargs):

    
    
    wall_follower = Node(
        package="ros2_wall_follower",
        executable="wall_follower.py", # py version
        name="wall_follower",
        output="screen",
        emulate_tty=True,
        remappings=[
            ('/scan', '/ldlidar_node/scan'),
            ('/cmd_vel', '/cmd_vel_nav')
        ]
    )

    return [
        wall_follower,
    ]

def generate_launch_description():
    return LaunchDescription([
        
        # Launch arguments
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),
        OpaqueFunction(function=launch_setup)
    ])