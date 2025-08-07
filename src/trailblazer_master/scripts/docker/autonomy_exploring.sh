#!/bin/bash

source /opt/ros/humble/setup.bash

cd /home/legendary/TrailblazerML
# Uruchom kontener (jeśli był zatrzymany)
docker start trb_4_arm_active

# Wykonaj polecenia wewnątrz kontenera
docker exec -it trb_4_arm_active bash -c '
  export FASTRTPS_DEFAULT_PROFILES_FILE=/root/fastdds.xml
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch trailblazer_bringup anatolian_tasks_sequencer.launch.py
'

echo "wykonano zadanie"
