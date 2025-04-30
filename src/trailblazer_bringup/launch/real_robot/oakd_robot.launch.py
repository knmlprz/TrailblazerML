import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
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

    oak_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('trailblazer_oak'), 'launch', 'depthai.launch.py')
        )
    )

    pkg_nav2 = get_package_share_directory('trailblazer_nav2')
    ekf_config_path = PathJoinSubstitution([
        pkg_nav2, 'config', 'ekf.yaml'
    ])

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('trailblazer_rviz'), 'launch', 'rviz.launch.py')
        ),
        launch_arguments={'rviz_config': rviz_config_path}.items()
    )

    return LaunchDescription([
        rsp,
        controller_launch,
        oak_camera_launch,
        ekf_node,
        rviz_launch
    ])