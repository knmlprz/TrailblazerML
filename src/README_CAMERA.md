The following script will install depthai-core and update usb rules and install depthai devices:

sudo wget -qO- https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh | sudo bash

if you don't have opencv installed then try "sudo apt install libopencv-dev"

sudo apt install python3-rosdep
sudo rosdep init
rosdep update

jeśli pobrałeś pliki z tego brancha nie musisz pobierać repozytorium luxonis
jeśli chcesz postawić od nowa to tworzysz katalog workspace/src i w src pobierasz repozytorium
git clone --branch humble https://github.com/luxonis/depthai-ros.git
cd ..

rosdep install --from-paths src --ignore-src -r -y

colcon build

source install/setup.bash


Gdy chcesz odpalić jakiś przykładowy program z publikowaniem topiców z kamer:
w paczkach depthai_ros_driver i depthai_examples znajdują się pliki launch które uruchamiają kamerkę. 

// publikuje rgb_camera stereo_camera i inne
ros2 launch depthai_ros_driver camera.launch.py

// publikuje stereo, rgb, pointcloud, imo można włączać w configu 
ros2 launch depthai_ros_driver pointcloud.launch.py 


Aby uruchomić aruco trzeba:
sudo apt install ros-humble-aruco-opencv ros-humble-aruco-opencv-msgs

wejsc do workspace i wykonać: concol build

uruchamiamy:
1 terminal) ros2 launch depthai_ros_driver aruco_camera.launch.py
2 terminal) ros2 launch aruco_opencv aruco_tracker.launch.xml

możemy w rviz obserwoawać image na topicu /aruco_tracker/debug i będziemy mieć podgląd

