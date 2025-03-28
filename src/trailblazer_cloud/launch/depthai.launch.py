import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    parameters = [{
        'frame_id': 'base_link',
        'subscribe_rgbd': True,
        'subscribe_scan': True,  # Włącz subskrypcję danych z lidaru
        'subscribe_odom_info': False,
        'approx_sync': True,  # Zmień na True, aby synchronizować różne źródła
        'wait_imu_to_init': False,
        'Reg/Strategy': '1',  # 1=ICP dla lidaru, 0=wizualne, 2=hybrydowe
        'Icp/VoxelSize': '0.05',  # Rozmiar woksela dla ICP
        'Icp/MaxCorrespondenceDistance': '0.1',  # Maksymalna odległość dla ICP
        'Reg/Force3DoF':'true',
        "RGBD/NeighborLinkRefining": 'true',
        'RGBD/ProximityBySpace': 'true',  # Wykrywanie zamknięć pętli z lidarem
        "RGBD/AngularUpdate": '0.01',
        "RGBD/LinearUpdate": '0.01',
        "RGBD/OptimizeFromGraphEnd": 'false',
        'Grid/FromDepth': 'false',  # Mapa zajętości z lidaru, nie z głębi
        "Rtabmap/DetectionRate": "3",
        'Odom/ResetCountdown': '10',
        'Mem/RehearsalSimilarity': '0.45',
        "Grid/Sensor": "0",
        'RGBD/ProximityPathMaxNeighbors': "10",

    }]

    remappings = [
        ('imu', '/imu/data'),
        ('scan', '/ldlidar_node/scan'),  # Mapowanie tematu lidaru
        ('rgb/image', '/right/image_rect'),
        ('rgb/camera_info', '/right/camera_info'),
        ('depth/image', '/stereo/depth'),
    ]

    return LaunchDescription([
        # Launch camera driver (DepthAI)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('depthai_examples'), 'launch'),
                '/stereo_inertial_node.launch.py']),
            launch_arguments={
                'depth_aligned': 'false',
                'enableRviz': 'false',
                'monoResolution': '400p',
                'enableDotProjector': 'true',
                'enableFloodLight': 'true'
            }.items(),
        ),

       
        # Sync right/depth/camera_info together
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            output='screen',
            parameters=parameters,
            remappings=[('rgb/image', '/right/image_rect'),
                        ('rgb/camera_info', '/right/camera_info'),
                        ('depth/image', '/stereo/depth')]),

        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': True
            }],
            remappings=[('imu/data_raw', '/imu')]),

        # Visual odometry with RGBD
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            output='screen',
            parameters=[parameters[0],{
                'publish_tf': True}],
            remappings=remappings),

        # VSLAM with RGBD and lidar
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        # Visualization
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=parameters,
            remappings=remappings),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'oak-d-base-frame']
        )
    ])