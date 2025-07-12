ros2 launch ros2_wall_follower wall_follower_py_remap.launch.py 
ros2 launch ldlidar_node ldlidar_rviz2.launch.py
rviz2 -d src/ros2_wall_follower/rviz/wall_follower.rviz 

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_nav
ros2 run ros2_wall_follower aruco_searching.py 
cd ~/ros2_ws_aruco
source install/setup.bash
ros2 run aruco_opencv aruco_tracker_autostart --ros-args -p cam_base_topic:=oak/rgb/image_raw -p marker_size:=0.15 -p marker_dict:=ARUCO_ORIGINAL
ros2 launch depthai_ros_driver camera.launch.py 
ros2 service call /aruco_searching_start std_srvs/srv/Trigger
```