```bash
ros2 launch trailblazer_bringup oakd_robot.launch.py 
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false
ros2 launch nav2_bringup rviz_launch.py 
```