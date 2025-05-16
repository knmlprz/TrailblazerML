```bash
ros2 launch trailblazer_bringup camera_robot.launch.py 
ros2 launch trailblazer_nav2 dual_ekf_navsat.launch.py 
ros2 topic pub --rate 10 /gps/fix sensor_msgs/NavSatFix "{
  header: { stamp: { sec: $(date +%s), nanosec: 0 }, frame_id: 'base_link' },
  latitude: 0.0411,
  longitude: 0.0001,
  altitude: 0.0,
  status: { status: 1, service: 1 }
}"
ros2 run trailblazer_nav2 logged_waypoint_follower 

```