# Trailblazer ArUco Detection

ROS 2 package that launches a DepthAI camera and detects ArUco markers using `aruco_opencv`.

## Requirements

- ROS 2 Humble
- [depthai_ros_driver](https://github.com/luxonis/depthai-ros)
- `vision_opencv` and `ros_aruco_opencv` in your workspace

## Installation

Clone dependencies as submodules:

```bash
git submodule update --init --recursive
rosdep install --from-paths src --ignore-src -r -y
colcon build --allow-overriding cv_bridge image_geometry

# USAGE 

## How to run
ros2 launch trailblazer_aruco_detection aruco_pipeline_launch.py

## How to test
ros2 topic echo /aruco_detections
