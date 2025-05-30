from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def launch_setup(context, *args, **kwargs):
    # --- PACKAGES ---
    pkg_trailblazer_cloud = get_package_share_directory('trailblazer_cloud')
    pkg_trailblazer_description = get_package_share_directory('trailblazer_description')
    pkg_trailblazer_controller = get_package_share_directory('trailblazer_controller')
    pkg_trailblazer_nav2 = get_package_share_directory('trailblazer_nav2')
    pkg_ekf_bringup = get_package_share_directory('trailblazer_ekf')
    pkg_gps_bringup = get_package_share_directory('trailblazer_gps')

    # --- PARAMS PATHS ---
    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare('trailblazer_nav2'), 'config', 'champ_nav2_params.yaml']
    )

    # --- LAUNCH FILES ---
    nav2_launch_path = PathJoinSubstitution([pkg_trailblazer_nav2, 'launch', 'navigation_launch.py'])
    rtabmap_launch_path = PathJoinSubstitution([pkg_trailblazer_cloud, 'launch', 'depthai_without_vo.launch.py'])
    ekf_navsat_launch_path = PathJoinSubstitution([pkg_ekf_bringup, 'launch', 'ekf.launch.py'])
    rsp_launch_path = PathJoinSubstitution([pkg_trailblazer_description, 'launch', 'rsp.launch.py'])
    controller_launch_path = PathJoinSubstitution([pkg_trailblazer_controller, 'launch', 'controller.launch.py'])

    # --- STATIC TRANSFORMS ---
    odom_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'] # arguments=['0', '0', '0', '1.57', '0', '0', 'odom', 'base_link']
    )

    # NODEs
    gps_node = Node(
        package='trailblazer_gps',
        executable="rtk_gps",
        name='gps_node',
        parameters=[{
            'port': "/dev/ttyUSB0",
        }]
    )

    interactive_waypoint_follower_node = Node(
        package='trailblazer_nav2',
        executable="interactive_waypoint_follower",
        name='interactive_waypoint_follower_node',
    )

    # --- INCLUDE LAUNCH TO RUN ---
    # NAV2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_path]),
        launch_arguments=[
            ('use_sim_time', 'false'),
            ('params_file', nav2_params_file)
        ]
    )

    # RTABMAP
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rtabmap_launch_path]),
        launch_arguments=[
            ('localization', LaunchConfiguration('localization')),
            ('use_sim_time', 'false'),
            ('use_ros2_control', 'true')
        ]
    )

    # EKF + NAVSAT
    ekf_navsat_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ekf_navsat_launch_path]))

    # ROBOT STATE PUBLISHER
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rsp_launch_path]),
        launch_arguments=[
            ('use_sim_time', 'false'),
            ('use_ros2_control', 'true')
        ])

    # WHEELS CONTROLLER
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([controller_launch_path]))

    return [
        nav2_launch,
        rtabmap_launch,
        rsp_launch,
        controller_launch,
        ekf_navsat_launch,
        gps_node,
        interactive_waypoint_follower_node,
    ]

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'
        ),
        OpaqueFunction(function=launch_setup)
    ])