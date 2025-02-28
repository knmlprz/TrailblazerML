import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('trailblazer_description'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # Node TF publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description_config,
                'ignore_timestamp': True
            }
        ]
    )

    # Node Joint State publisher
    node_robot_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[
            {
                'rate':10,
                'use_gui': False,
                'source_list': ['/my_robot/joint_states'] # można dodać dodatkowe topic na którym joint_state będzie nasłuchiwał
            }
        ]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_robot_joint_state_publisher
    ])
