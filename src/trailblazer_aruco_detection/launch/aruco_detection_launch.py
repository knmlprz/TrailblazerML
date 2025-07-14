from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('depthai_ros_driver'),
                'launch',
                'camera.launch.py'
            )
        ])
    )

    aruco_tracker = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        name='aruco_tracker',
        parameters=[{
            'cam_base_topic': 'oak/rgb/image_raw',
            'marker_size': 0.15,
            'marker_dict': 'ARUCO_ORIGINAL'
        }]
    )

    aruco1_publisher = Node(
        package='trailblazer_aruco_detection',
        executable='aruco1_publisher'
    )

    aruco2_publisher = Node(
        package='trailblazer_aruco_detection',
        executable='aruco2_publisher'
    )

    return LaunchDescription([
        camera_launch,
        aruco_tracker,
        aruco1_publisher,
        aruco2_publisher
    ])
