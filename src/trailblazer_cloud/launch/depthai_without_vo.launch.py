import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("mxId", default_value="x"),
        DeclareLaunchArgument("depth_aligned", default_value="false")
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration("name").perform(context)
    depth_aligned = LaunchConfiguration("depth_aligned").perform(context)

    # RTAB-Map Parameters
    vslam_params = {
        'frame_id': 'base_link',
        'guess_frame_id': 'odom',
        'approx_sync': True,
        'use_sim_time': False,
        'subscribe_rgbd': True,
        'subscribe_odom_info': False,
        'use_action_for_goal': True,
        'wait_imu_to_init': False,
        'wait_for_transform': 0.5,
        'Grid/DepthDecimation': '1',
        'Grid/RangeMax': '2',
        'GridGlobal/MinSize': '20',
        'Grid/MinClusterSize': '20',
        'Grid/MaxObstacleHeight': '2',
        'Odom/ResetCountdown': '2',
        'Kp/RoiRatios': '0.0 0.0 0.0 0.4'
    }

    vslam_remappings = [
        ('imu', 'imu/data'),
        ('odom', '/diff_drive_controller/odom')
    ]

    nodes = []

    # Kamera (DepthAI ROS stereo + IMU)
    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('trailblazer_cloud'),
                    'launch/stereo_inertial_node.launch.py')
            ]),
            launch_arguments={
                'name': name,
                'depth_aligned': depth_aligned,
                'enableDotProjector': 'true',
                'enableFloodLight': 'true',
                'monoResolution': '400p',
            }.items(),
        )
    )

    # Synchronizacja RGBD (dwa warianty zależnie od wyrównania głębokości)
    if depth_aligned == "true":
        rgb_remap = [
            ('rgb/image', '/color/image'),
            ('rgb/camera_info', '/color/camera_info'),
            ('depth/image', '/stereo/depth')
        ]
    else:
        rgb_remap = [
            ('rgb/image', '/right/image_rect'),
            ('rgb/camera_info', '/right/camera_info'),
            ('depth/image', '/stereo/depth')
        ]

    nodes.append(
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            output='screen',
            parameters=[vslam_params],
            remappings=rgb_remap
        )
    )

    # RTAB-Map główny
    nodes.append(
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[vslam_params],
            remappings=vslam_remappings,
            arguments=['-d']
        )
    )

    # Chmura punktów
    nodes.append(
        Node(
            package='rtabmap_util',
            executable='point_cloud_xyz',
            output='screen',
            parameters=[{
                'decimation': 2,
                'max_depth': 3.0,
                'voxel_size': 0.02
            }],
            remappings=[
                ('depth/image', '/stereo/depth'),
                ('depth/camera_info', '/stereo/camera_info'),
                ('cloud', '/camera/cloud')
            ]
        )
    )

    # Detekcja przeszkód
    nodes.append(
        Node(
            package='rtabmap_util',
            executable='obstacles_detection',
            output='screen',
            parameters=[vslam_params],
            remappings=[
                ('cloud', '/camera/cloud'),
                ('obstacles', '/camera/obstacles'),
                ('ground', '/camera/ground')
            ]
        )
    )

    return nodes
