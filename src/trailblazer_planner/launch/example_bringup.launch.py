from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #Node(
            #package='trailblazer_planner',
            #executable='service_example.py',
            #name='service_example',
            #output='screen'
        #),
        Node(
            package='trailblazer_planner',
            executable='example_node_1_2.py',
            name='example_node_1_2',
            output='screen'
        ),
        Node(
            package='trailblazer_planner',
            executable='example_node_3_4.py',
            name='example_node_3_4',
            output='screen'
        ),
        #Node(
            #package='trailblazer_planner',
            #executables='aruco_searching.py',
            #name='aruco_searching',
            #output='screen'
        #),

        #Node(
            #package='trailblazer_planner',
            #executables='driving_to_aruco.py',
            #name='driving_to_aruco',
            #output='screen'
        #),
        #Node(
            #package='trailblazer_planner',
            #executables='wall_follower.py',
            #name='wall_follower',
            #output='screen'
        #)
    ])