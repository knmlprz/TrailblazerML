#!/bin/bash

source /opt/ros/humble/setup.bash

cd /home/rafal/TrailblazerML
# Uruchom kontener (jeśli był zatrzymany)
source install/setup.bash
ros2 launch trailblazer_bringup all_lidar_wall_follower.launch.py

echo "wykonano zadanie"
