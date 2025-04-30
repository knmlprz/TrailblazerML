import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    fake_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('trailblazer_rviz'), 'config', 'controller.rviz'
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('trailblazer_description'), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('trailblazer_controller'), 'launch', 'controller.launch.py')
        )
    )

    joy_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('trailblazer_joystick'), 'launch', 'joy_control.launch.py')
        ),launch_arguments={
            'cmd_vel_topic': '/diff_drive_controller/cmd_vel'
        }.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('trailblazer_rviz'), 'launch', 'rviz.launch.py')
        ),
        launch_arguments={'rviz_config': rviz_config_path}.items()
    )

    return LaunchDescription([
        fake_odom,
        rsp,
        controller_launch,
        joy_control_launch,
        rviz_launch
    ])