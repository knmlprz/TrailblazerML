from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction
from launch_ros.substitutions import FindPackageShare
import os

def launch_setup(context, *args, **kwargs):
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('depthai_ros_driver'),
                'launch',
                'camera.launch.py'
            )
        )
    )
    camera_to_link_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'oak-d-base-frame']
    )
    aruco_detection = Node(
            package='aruco_opencv',
            executable='aruco_tracker_autostart',
            name='aruco_tracker',
            output='screen',
            parameters=[
                {"cam_base_topic": "oak/rgb/image_raw"},
                {"marker_size": 0.15},
                {"marker_dict": "ARUCO_ORIGINAL"}
            ]
        )
    
    return [
        camera_launch,
        camera_to_link_transform,
        aruco_detection,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
