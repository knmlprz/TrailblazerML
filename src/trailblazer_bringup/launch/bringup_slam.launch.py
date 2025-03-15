import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    # Ścieżki do plików konfiguracyjnych
    lc_mgr_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'), 'params', 'lifecycle_mgr_slam.yaml'
    )
    slam_config_path = os.path.join(
        get_package_share_directory('ldlidar_node'), 'params', 'slam_toolbox.yaml'
    )
    rviz_config_path = os.path.join(
        get_package_share_directory('trailblazer_rviz'), 'config', 'slam.rviz'
    )

    # Węzeł menedżera cyklu życia
    lc_mgr_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[lc_mgr_config_path]
    )

    # Węzeł SLAM Toolbox w trybie async
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        namespace='',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_path],
        remappings=[('/scan', '/ldlidar_node/scan')]
    )

    fake_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'laser_frame', 'ldlidar_base']
    )

    # fake odom
    fake_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    # Uruchomienie opisu robota
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('trailblazer_bringup'), 'launch', 'bringup_controllers.launch.py')
        )
    )

    # Uruchomienie LIDARa
    ldlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ldlidar_node'), 'launch', 'ldlidar_bringup.launch.py')
        ),
        launch_arguments={'node_name': 'ldlidar_node'}.items()
    )

    # Uruchomienie RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('trailblazer_rviz'), 'launch', 'rviz.launch.py')
        ),
        launch_arguments={'rviz_config': rviz_config_path}.items()
    )

    # Definicja LaunchDescription
    ld = LaunchDescription()
    ld.add_action(lc_mgr_node)
    ld.add_action(slam_toolbox_node)
    ld.add_action(fake_laser)
    ld.add_action(fake_odom)
    ld.add_action(description_launch)
    ld.add_action(ldlidar_launch)
    ld.add_action(rviz_launch)

    return ld
