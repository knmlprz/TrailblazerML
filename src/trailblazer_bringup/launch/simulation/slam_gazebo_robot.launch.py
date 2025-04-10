import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('trailblazer_description'), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()
    )

    # World configuration
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('trailblazer_gazebo'),
            'worlds',
            'office.world'
        ),
        description='Path to world file to load in Gazebo'
    )

    # Gazebo configuration
    gazebo_params_file = os.path.join(
        get_package_share_directory('trailblazer_gazebo'),
        'config',
        'gazebo_params.yaml'
    )

    # Gazebo launch with world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
            'world': LaunchConfiguration('world')
        }.items()
    )

    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'rover_legendary'],
        output='screen'
    )

    # Joystick control
    joy_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('trailblazer_joystick'), 'launch', 'joy_control.launch.py')
        ),
        launch_arguments={
            'cmd_vel_topic': '/cmd_vel'
        }.items()
    )

    # SLAM Toolbox
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('trailblazer_slam'), 'launch', 'slam.launch.py')
        )
    )

    # RViz configuration
    rviz_config_path = os.path.join(
        get_package_share_directory('trailblazer_rviz'), 'config', 'simulation.rviz'
    )
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('trailblazer_rviz'), 'launch', 'rviz.launch.py')
        ),
        launch_arguments={'rviz_config': rviz_config_path}.items()
    )

    return LaunchDescription([
        world_arg,
        rsp,
        gazebo,
        spawn_entity,
        rviz_launch,
        joy_control_launch,
        slam_toolbox_launch
    ])