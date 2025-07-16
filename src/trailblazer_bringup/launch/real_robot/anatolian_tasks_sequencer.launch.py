from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import os

def launch_setup(context, *args, **kwargs):
    service_sequencer_launch = Node(
            package='trailblazer_planner',
            executable='anatolian_service_sequencer.py',
            name='service_sequencer',
            output='screen'
        )


    return [
        service_sequencer_launch,
    ]

def generate_launch_description():
    return LaunchDescription([
        
        # Launch arguments
        OpaqueFunction(function=launch_setup)
    ])