#!/bin/bash

source /opt/ros/humble/setup.bash

cd /home/legendary/TrailblazerML
# Uruchom kontener i wykonaj komendy wewnątrz niego
docker start trb_4_arm2

docker exec -it trb_4_arm2 /bin/bash -c "
  source install/setup.bash && \
  ros2 launch trailblazer_bringup all_lidar_wall_follower.launch.py
"
