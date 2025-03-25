# Requirements:
#   A OAK-D camera
#   Install depthai-ros package (https://github.com/luxonis/depthai-ros)
# Example:
#   $ ros2 launch rtabmap_examples depthai.launch.py camera_model:=OAK-D

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    parameters=[{'frame_id':'base_link',
                 'subscribe_rgbd':True,
                 'subscribe_odom_info':True,
                 #'approx_sync':True,
                 'wait_imu_to_init':True,
                 'Reg/Force3DoF':             'true',    # 2D SLAM
                "Rtabmap/DetectionRate": "1",
                'Odom/ResetCountdown': '10',
                'Mem/RehearsalSimilarity': '0.45',

                'approx_sync': False,
                'use_action_for_goal':True,
                'Reg/Force3DoF':'true',
                'Vis/MinDepth': '0.2',
                'GFTT/MinDistance': '5',
                'GFTT/QualityLevel': '0.00001',
                'Grid/RayTracing':'true', # Fill empty space
                'Grid/3D':'false', # Use 2D occupancy
                'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
                'Grid/MaxGroundHeight':'0.15', # All points above 5 cm are obstacles
                'Grid/MaxObstacleHeight':'0.5',  # All points over 0.5 meter are ignored
                'Grid/RangeMin':'0.2',  # Ignore invalid points close to camera
                'Grid/NoiseFilteringMinNeighbors':'8',  # Default stereo is quite noisy, enable noise filter
                'Grid/NoiseFilteringRadius':'0.1',  # Default stereo is quite noisy, enable noise filter
                'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
                #'publish_tf': False,
                #'odom_frame_id': 'odom',
                # 'subscribe_odom':True,
                # 'odom_topic': '/odom', 
                # 'Reg/Strategy': '1'
                }]

    remappings=[('imu', '/imu/data')]

    return LaunchDescription([

        # Launch camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('depthai_examples'), 'launch'),
                '/stereo_inertial_node.launch.py']),
                launch_arguments={'depth_aligned': 'false',
                                  'enableRviz': 'false',
                                  'monoResolution': '400p',
                                  'extended': 'true',
                                  'enableDotProjector': 'true',
                                  'enableFloodLight': 'true'
                                  }.items(),
        ),

        # Sync right/depth/camera_info together
        Node(   
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=parameters,
            remappings=[('rgb/image', '/right/image_rect'),
                        ('rgb/camera_info', '/right/camera_info'),
                        ('depth/image', '/stereo/depth')]),

        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/imu')]),

        # Visual odometry
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        # VSLAM
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        # Visualization
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings)
    ])
