import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Ścieżki do pakietów
    pkg_trailblazer_cloud = get_package_share_directory('trailblazer_cloud')
    pkg_trailblazer_description = get_package_share_directory('trailblazer_description')
    pkg_trailblazer_controller = get_package_share_directory('trailblazer_controller')
    pkg_trailblazer_nav2 = get_package_share_directory('trailblazer_nav2')
    pkg_trailblazer_rviz = get_package_share_directory('trailblazer_rviz')

    # Ścieżki do plików konfiguracyjnych
    rviz_config_path = os.path.join(pkg_trailblazer_rviz, 'config', 'controller.rviz')
    ekf_config_path = os.path.join(pkg_trailblazer_nav2, 'config', 'ekf.yaml')
    nav2_config_path = PathJoinSubstitution([pkg_trailblazer_nav2, 'config', 'trailblazer_rgbd_nav2_params.yaml'])

    # Launch robot_state_publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_trailblazer_description, 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    # Launch controllers
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_trailblazer_controller, 'launch', 'controller.launch.py')
        )
    )

    # Launch RTAB-Map or DepthAI
    rtabmap_launch = PathJoinSubstitution(
        [pkg_trailblazer_cloud, 'launch', 'depthai.launch.py'])
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rtabmap_launch]),
        launch_arguments=[
            ('localization', LaunchConfiguration('localization')),
            ('use_sim_time', 'false'),
            ('use_ros2_control', 'true')
        ]
    )

    # gps
    gps_node = Node(
        package='trailblazer_gps',
        executable="rtk_gps",
        name='gps_node',
        parameters=[{
            'port': "/dev/ttyUSB0",
        }]
    )

    # EKF odometry
    ekf_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': False}],
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    # EKF map (global pose estimation)
    ekf_map = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': False}],
        remappings=[('odometry/filtered', 'odometry/global')]
    )

    # NavSat Transform node
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': False}],
        remappings=[
            ('/imu/data_cov', 'imu/data_cov'),
            ('gps/fix', 'gps/fix'),
            ('gps/filtered', 'gps/filtered'),
            ('odometry/gps', 'odometry/gps'),
            ('odometry/filtered', 'odometry/global')
        ]
    )

    # RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_trailblazer_rviz, 'launch', 'rviz.launch.py')
        ),
        launch_arguments={'rviz_config': rviz_config_path}.items()
    )

    # NAV2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('trailblazer_nav2'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'use_sim_time': 'false', 'params_file': nav2_config_path, 'autostart': 'true'}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'
        ),
        rsp,
        controller_launch,
        rtabmap,
        gps_node,
        ekf_odom,
        ekf_map,
        navsat_transform,
        # nav2,
        rviz_launch
    ])
