from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import os

def launch_setup(context, *args, **kwargs):

    aruco_searching = Node(
        package="trailblazer_planner",
        executable="aruco_searching.py", # py version
        name="aruco_searching_node",
        output="screen",
        emulate_tty=True,
    )

    driving_to_aruco = Node(
        package="trailblazer_planner",
        executable="driving_to_aruco.py", # py version
        name="driving_to_aruco_node",
        output="screen",
        emulate_tty=True,
    )

    wall_follower = Node(
        package="trailblazer_planner",
        executable="wall_follower.py", # py version
        name="wall_follower_node",
        output="screen",
        emulate_tty=True,
        remappings=[
            ('/scan', '/ldlidar_node/scan'),
            ('/cmd_vel', '/cmd_vel_nav')
        ]
    )


    return [
        aruco_searching,
        driving_to_aruco,
        wall_follower,
    ]

def generate_launch_description():
    return LaunchDescription([
    
        # Launch arguments
        OpaqueFunction(function=launch_setup)
    ])