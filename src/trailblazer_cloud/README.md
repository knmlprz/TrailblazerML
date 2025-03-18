## Dependencies

### for clear install, remove old packages

```bash
sudo apt remove ros-humble-rtabmap*

sudo apt remove ros-humble-depthai*
```

### installation

```bash
sudo apt install ros-humble-depthai*

sudo apt install ros-humble-rtabmap*

```

### testing, after building
```bash
ros2 launch trailblazer_cloud depthai.launch.py
```

### added params explanation
```python
'monoResolution': '400p', 
#default value 720, If the camera has too much errors (red screen), set it to 400

'extended': 'true', 
#default false, set to true to enable

'enableDotProjector': 'true', 
#set this to true to enable the dot projector structure light (Available only on Pro models).

'enableFloodLight': 'true'
#Set this to true to enable the flood light for night vision (Available only on Pro models).'

'Reg/Force3DoF':             'true', 
#Set to true to disable "z" axis (up,down)

"Rtabmap/DetectionRate": "1",
#default 1, Set number for creating more or less picture

'Odom/ResetCountdown': '10',
#Default 0(disable), after 10 hz errors (red screan), it resets odometry

'Mem/RehearsalSimilarity': '0.45',
#Default 0(disable), after detecting loop closure with 0.45 disparity, merge pointclouds 
```