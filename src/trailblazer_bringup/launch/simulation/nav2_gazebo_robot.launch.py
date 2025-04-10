import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Ścieżka do pliku ekf.yaml
    ekf_config_path = os.path.join(
        get_package_share_directory('trailblazer_nav2'),
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('trailblazer_gazebo'),
                    'launch',
                    'simulation.launch.py'
                )
            ]),
            launch_arguments={'world': "office.world"}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('trailblazer_joystick'),
                    'launch',
                    'joy_control.launch.py'
                )
            ]),
            launch_arguments={'cmd_vel_topic': '/cmd_vel'}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('trailblazer_slam'),
                    'launch',
                    'slam.launch.py'
                )
            ])
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[ekf_config_path]  # Wymaga poprawnego pliku ekf.yaml
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('trailblazer_rviz'),
                    'launch',
                    'rviz.launch.py'
                )
            ]),
            launch_arguments={
                'rviz_config': os.path.join(
                    get_package_share_directory('trailblazer_rviz'),
                    'config',
                    'nav2.rviz'
                )
            }.items()
        )
    ])