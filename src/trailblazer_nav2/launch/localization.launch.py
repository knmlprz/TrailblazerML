import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('trailblazer_rviz'), 'config', 'nav2.rviz'
    )

    nav2_params = os.path.join(get_package_share_directory('trailblazer_nav2'), 'config', 'nav2_params.yaml')

    # Uruchomienie opisu robota
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('trailblazer_bringup'), 'launch', 'bringup_controllers.launch.py')
        )
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('trailblazer_nav2'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'use_sim_time': 'false', 'params_file': nav2_params}.items()
    )

    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('trailblazer_nav2'), 'launch', 'localization_launch.py')
        ),
        launch_arguments={'use_sim_time': 'false', 'params_file': nav2_params}.items()
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[nav2_localization])

    # Uruchomienie RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('trailblazer_rviz'), 'launch', 'rviz.launch.py')
        ),
        launch_arguments={'rviz_config': rviz_config_path}.items()
    )

    # Definicja LaunchDescription
    ld = LaunchDescription()
    ld.add_action(description_launch)
    ld.add_action(nav2)
    ld.add_action(nav2_localization)
    ld.add_action(rviz_launch)

    return ld
