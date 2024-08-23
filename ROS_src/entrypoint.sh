#!/bin/bash
set -e


# Źródłujemy środowisko ROS 2
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 pkg create --build-type ament_python src/my_package1
mv /ros2_ws/src/pack1/my_first_node.py /ros2_ws/src/my_package1/src/my_first_node.py
mv /ros2_ws/src/pack1/setup.py /ros2_ws/src/my_package1/setup.py
## Uruchamiamy nody
exec "$@"