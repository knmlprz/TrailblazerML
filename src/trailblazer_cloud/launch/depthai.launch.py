# Source: https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/turtlebot3/turtlebot3_rgbd.launch.py
# Requirements:
#   A OAK-D camera
#   Install depthai-ros package (https://github.com/luxonis/depthai-ros)
# Example:
#   $ ros2 launch trailblazer_cloud depthai.launch.py camera_model:=OAK-D

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Definiowanie argumentu launch dla pliku parametrów

    # Parametry dla RTAB-Map
    parameters = [{
        'frame_id': 'oak-d-base-frame',
        'subscribe_rgbd': True,
        'subscribe_odom_info': True,
        'approx_sync': True,
        'wait_imu_to_init': True,
        'use_action_for_goal': True,
        'Reg/Force3DoF': 'true',
        'Grid/RayTracing': 'true',  # Fill empty space
        'Grid/3D': 'false',  # Use 2D occupancy
        'Grid/RangeMax': '3',
        'Grid/NormalsSegmentation': 'false',  # Use passthrough filter to detect obstacles
        'Grid/MaxGroundHeight': '0.05',  # All points above 5 cm are obstacles
        'Grid/MaxObstacleHeight': '0.4',  # All points over 1 meter are ignored
        'Optimizer/GravitySigma': '0',  # Disable imu constraints (we are already in 2D)
        'Odom/ResetCountdown': '1', #def 0
        'Rtabmap/StartNewMapOnLoopClosure': 'true', #def false (set to true for navigating)
        'Odom/Strategy': '1', #def 0
        'Vis/CorType': '1', #def 0
        'OdomF2M/MaxSize': '1000', #def 2000
        'Vis/MaxFeatures': '600', #def 1000
        'GFTT/MinDistance': '10', #def 7
        'Grid/NoiseFilteringMinNeighbors': '2', #def 5
        'Grid/NoiseFilteringRadius': '0.05', #def 0
        'Grid/MinClusterSize': '20', #def 10

    }]

    remappings = [('imu', '/imu/data')]

    return LaunchDescription([

        # Launch camera driver z parametrami z YAML
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('trailblazer_cloud'), 'launch'),
                '/stereo_inertial_node.launch.py']),
                launch_arguments={
                    'LRchecktresh': '5',
                    'angularVelCovariance': '0.0',
                    'confidence': '200',
                    'depth_aligned': 'false',
                    'detectionClassesCount': '80',
                    'dotProjectorIntensity': '0.5',
                    'enableDotProjector': 'true',
                    'enableFloodLight': 'true',
                    'enableRosBaseTimeUpdate': 'false',
                    'enableSpatialDetection': 'true',
                    'expTime': '20000',
                    'extended': 'false',
                    'floodLightIntensity': '0.5',
                    'imuMode': '1',
                    'linearAccelCovariance': '0.0',
                    'lrcheck': 'true',
                    'manualExposure': 'false',
                    'mode': 'depth',
                    'monoResolution': '400p',
                    'mxId': '1944301041FCD22600',
                    'nnName': 'x',
                    'poeMode': 'false',
                    'previewHeight': '416',
                    'previewWidth': '416',
                    'rectify': 'true',
                    'resourceBaseFolder': 'x',
                    'rgbResolution': '1080p',
                    'rgbScaleDinominator': '3',
                    'rgbScaleNumerator': '2',
                    'sensIso': '800',
                    'stereo_fps': '60', #def 30
                    'subpixel': 'true',
                    'syncNN': 'true',
                    'tf_prefix': 'oak',
                    'usb2Mode': 'false',
                    'use_sim_time': 'false',
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
                         'world_frame': 'enu',
                         'publish_tf': False}],
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
            remappings=remappings),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'oak-d-base-frame', 'oak_imu_frame']
        ),
    ])