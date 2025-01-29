from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            remappings=[
                ('/cmd_vel', '/my_bot_cmd_vel')
            ],
        ),
    ])
