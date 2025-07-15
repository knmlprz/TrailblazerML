from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trailblazer_planner',
            executables='aruco_searching.py',
            name='aruco_searching',
            output='screen'
        ),

        Node(
            package='trailblazer_planner',
            executables='driving_to_aruco.py',
            name='driving_to_aruco',
            output='screen'
        ),
        Node(
            package='trailblazer_planner',
            executables='wall_follower.py',
            name='wall_follower',
            output='screen'
        )
    ])