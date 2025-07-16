from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trailblazer_planner',
            executable='service_example.py',
            name='service_example',
            output='screen'
        ),
        Node(
            package='trailblazer_planner',
            executable='example_node_1_2.py',
            name='example_node_1_2',
            output='screen'
        ),
        Node(
            package='trailblazer_planner',
            executable='service_planner_task3.py',
            name='service_planner_task3',
            output='screen'
        ),
    ])