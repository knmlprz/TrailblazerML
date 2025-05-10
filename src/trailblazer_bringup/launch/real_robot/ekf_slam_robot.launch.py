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

    # GPS
    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('trailblazer_gps'), 'launch', 'gps.launch.py')
        ), 
        launch_arguments={'port': '/dev/ttyUSB0', 'gps_type': 'rtk_gps'}.items()
    )

    # EKF
    gps_wpf_dir = get_package_share_directory(
        "trailblazer_nav2")
    rl_params_file = os.path.join(
        gps_wpf_dir, "config", "dual_ekf_navsat_params.yaml")
    # ekf_launch = Node(
    #     package="robot_localization",
    #     executable="navsat_transform_node",
    #     name="navsat_transform",
    #     output="screen",
    #     parameters=[rl_params_file, {"use_sim_time": False}],
    #     remappings=[
    #         ("imu/data", "imu/data"),
    #         ("gps/fix", "gps/fix"),
    #         ("gps/filtered", "gps/filtered"),
    #         ("odometry/gps", "odometry/gps"),
    #         ("odometry/filtered", "odometry/global"),
    #     ],
    # )
    ekf_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('trailblazer_nav2'), 'launch', 'dual_ekf_navsat.launch.py')
            )
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
        gps_launch,
        ekf_launch,
        rviz_launch
    ])