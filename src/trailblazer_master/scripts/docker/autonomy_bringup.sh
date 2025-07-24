#!/bin/bash

source /opt/ros/humble/setup.bash

cd /home/legendary/TrailblazerML

# Upewnij się, że plik fastdds.xml jest dostępny w kontenerze

# Start kontenera
docker start trb_4_arm2

# Uruchom ROS2 z konfiguracją FastDDS
docker exec -it trb_4_arm2 bash -c '
  export FASTRTPS_DEFAULT_PROFILES_FILE=/root/fastdds.xml
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch trailblazer_bringup all_lidar_wall_follower.launch.py
'
