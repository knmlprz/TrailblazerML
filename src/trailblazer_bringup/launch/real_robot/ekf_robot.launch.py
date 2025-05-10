import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav2 = get_package_share_directory('trailblazer_nav2')
    pkg_rviz = get_package_share_directory('trailblazer_rviz')
    lc_mgr_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'), 'params', 'lifecycle_mgr.yaml'
    )
    laser_to_link_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'laser_frame', 'ldlidar_base']
    )

    rviz_config_path = os.path.join(
        pkg_rviz, 'config', 'controller.rviz'
    )

    nav2_config_path = PathJoinSubstitution([
        pkg_nav2, 'config', 'nav2_params_real_robot.yaml'
    ])

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
        launch_arguments={'port': '/dev/ttyUSB1', 'gps_type': 'rtk_gps'}.items()
    )

    # EKF
    gps_wpf_dir = get_package_share_directory(
        "trailblazer_nav2")
    rl_params_file = os.path.join(
        gps_wpf_dir, "config", "dual_ekf_navsat_params.yaml")
    
    # ekf_launch = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(get_package_share_directory('trailblazer_nav2'), 'launch', 'dual_ekf_navsat.launch.py')
    #         )
    #     )
    ekf_launch = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[rl_params_file, {"use_sim_time": False}],
        remappings=[
            ("imu/data", "imu/data"),
            ("gps/fix", "gps/fix"),
            ("gps/filtered", "gps/filtered"),
            ("odometry/gps", "odometry/gps"),
            ("odometry/filtered", "odometry/global"),
        ],
    )


    lc_mgr_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            # YAML files
            lc_mgr_config_path  # Parameters
        ]
    )

    ldlidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('ldlidar_node'),
            '/launch/ldlidar_bringup.launch.py'
        ]),
        launch_arguments={
            'node_name': 'ldlidar_node'
        }.items()
    )

    # # # SLAM
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('trailblazer_slam'), 'launch', 'slam.launch.py')
        )
    )

    # NAV2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('trailblazer_nav2'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true', 'params_file': nav2_config_path, 'autostart': 'true'}.items()
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
        laser_to_link_transform,
        lc_mgr_node,
        ldlidar_launch,
        slam_launch,
        # nav2_launch,
        rviz_launch
    ])