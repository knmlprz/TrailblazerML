import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'depth_camera'


    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py',
        )])
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('depthai_ros_driver'), 'launch', 'camera_as_part_of_a_robot.launch.py',
        )])
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=[ '-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')
    

    state_publisher = Node(
        package='joint_state_publisher_gui', 
        executable='joint_state_publisher_gui', 
        name='joint_state_publisher_gui', 
        output='screen'
    )

    return LaunchDescription([
        rsp,
        #gazebo,
        spawn_entity,
        camera,
        state_publisher,
    ])