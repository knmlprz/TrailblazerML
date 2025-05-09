from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyUSB0",
        description="Port USB, np. /dev/ttyUSB0"
    )

    gps_type_arg = DeclareLaunchArgument(
        "gps_type",
        default_value="rtk",
        description="Typ GPS: 'basic' lub 'rtk'"
    )

    gps_type = LaunchConfiguration("gps_type")
    port = LaunchConfiguration("port")

    return LaunchDescription([
        port_arg,
        gps_type_arg,

        Node(
            package="trailblazer_gps",
            executable="basic_gps" if gps_type.perform({}) == 'basic' else "gps_rtk",
            name="gps_publisher",
            parameters=[{"port": port}]
        )
    ])
