from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_teleop_twist_joy',
            executable='teleop_twist_node',
            name='teleop_twist_node',
            output='screen',
            parameters=[{
                'axis_linear_rt': 5,
                'axis_linear_lt': 2,
                'axis_angular': 3,
                'scale_linear': 2.0,
                'scale_angular': 2.0
            }]
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        )
    ])
