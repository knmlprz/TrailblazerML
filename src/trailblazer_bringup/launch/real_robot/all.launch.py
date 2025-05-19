from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import os

def launch_setup(context, *args, **kwargs):

    # Directories
    pkg_nav2_bringup = get_package_share_directory(
        'nav2_bringup')
    pkg_trailblazer_cloud = get_package_share_directory(
        'trailblazer_cloud')
    
    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare('trailblazer_cloud'), 'params', 'trailblazer_rgbd_nav2_params.yaml']
    )
    

    pkg_trailblazer_description = get_package_share_directory('trailblazer_description')
    pkg_trailblazer_controller = get_package_share_directory('trailblazer_controller')
    pkg_trailblazer_nav2 = get_package_share_directory('trailblazer_nav2')

    # Paths
    nav2_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
    rviz_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'rviz_launch.py'])
    rtabmap_launch = PathJoinSubstitution(
        [pkg_trailblazer_cloud, 'launch', 'depthai.launch.py'])
    rsp_launch = PathJoinSubstitution(
        [pkg_trailblazer_description, 'launch', 'rsp.launch.py'])
    controller_launch_path = PathJoinSubstitution(
        [pkg_trailblazer_controller, 'launch', 'controller.launch.py'])
    gps_launch_path = PathJoinSubstitution(
        [pkg_trailblazer_nav2, 'launch', 'dual_ekf_navsat.launch.py'])
    
    # Includes
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('use_sim_time', 'false'),
            ('params_file', nav2_params_file)
        ]
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch])
    )
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rtabmap_launch]),
        launch_arguments=[
            ('localization', LaunchConfiguration('localization')),
            ('use_sim_time', 'false'),
            ('use_ros2_control', 'true')
        ]
    )
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rsp_launch]),
        launch_arguments=[
            ('use_sim_time', 'false'),
            ('use_ros2_control', 'true')
        ])

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([controller_launch_path]))
    
    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gps_launch_path]))
    
    gps_node = Node(
        package='trailblazer_gps',
        executable="rtk_gps",
        name='gps_node',
        parameters=[{
            'port': "/dev/ttyUSB1",
        }]
    )
    return [
        # Nodes to launch
        nav2,
        rviz,
        rtabmap,
        rsp,
        controller_launch,
        gps_launch,
        gps_node
    ]

def generate_launch_description():
    return LaunchDescription([
        
        # Launch arguments
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),
        OpaqueFunction(function=launch_setup)
    ])