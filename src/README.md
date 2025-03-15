# TrailblazerML ROS2 Project

This repository contains a ROS2-based project using the `humble` distribution. The project integrates simulation, visualization, teleoperation, and machine learning for autonomous robot navigation and control.

---

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Installation](#installation)
   - [Install ROS2 Humble](#install-ros2-humble)
   - [Setup the Workspace](#setup-the-workspace)
3. [Running the Project](#running-the-project)
   - [Launching Simulation](#launching-simulation)
   - [Teleoperation](#teleoperation)
4. [Project Structure](#project-structure)

---

## Prerequisites
Ensure the following are installed on your system:
- Ubuntu 22.04
- Python 3.10+
- [ROS2 Humble](https://docs.ros.org/en/humble/index.html)
- `colcon` build tools
- Gazebo simulation environment

---

## Installation

### Install ROS2 Humble
Follow the official [ROS2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html).

### Setup the Workspace
1. Clone this repository:
    ```bash
    git clone https://github.com/knmlprz/TrailblazerML.git
    cd TrailblazerML
    ```
2. Install dependencies:
    ```bash
    sudo apt update && sudo apt install -y \
      python3-colcon-common-extensions \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-gazebo-ros2-control \
      ros-humble-position-controllers \
      ros-humble-controller-manager-spawner \
      ros-humble-xacro \
      joystick \
      jstest-gtk \
      evtest \
      ros-humble-twist-mux \
      ros-humble-rviz2
    ```
3. Build the workspace:
    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```

---

## Running the Project

## Add meshes to Gazebo
1. Go to gazebo folder:
```bash
   cd ~/.gazebo/models
   mkdir trailblazer_description
```
2. Add meshes:
```bash
  cp -r <Path_to_project>/TrailblazerML/src/trailblazer_description/meshes ./trailblazer_description
```

## Project Structure
```plaintext
src
├── ldrobot-lidar-ros2  # Pakiet sterownika LiDAR od LDROBOT
├── trailblazer_bringup # Pakiet uruchamiania robota
│   ├── launch
│   │   ├── bringup_controllers.launch.py    # Uruchamia kontrolery robota
│   │   ├── bringup_nav.launch.py            # Uruchamia nawigację z RViz
│   │   └── bringup_slam.launch.py           # Uruchamia SLAM z RViz
├── trailblazer_description # Opis robota w URDF
│   ├── config
│   │   └── controllers.yaml # Konfiguracja kontrolerów
│   ├── description
│   │   ├── camera.xacro
│   │   ├── depth_camera.xacro
│   │   ├── inertial_macros.xacro
│   │   ├── lidar.xacro
│   │   ├── robot_core.xacro
│   │   ├── robot.urdf.xacro
│   │   ├── ros2_control_gazebo.xacro
│   │   └── ros2_control.xacro
│   ├── launch
│   │   ├── __pycache__
│   │   └── rsp.launch.py # Uruchamia robot state publisher
│   ├── meshes
│   │   ├── body.stl
│   │   ├── wheel_left.stl
│   │   └── wheel_right.stl
├── trailblazer_gazebo # Pakiet symulacji w Gazebo
│   ├── config
│   │   └── controllers.yaml # Konfiguracja kontrolerów w symulacji
│   ├── launch
│   │   └── launch_sim.launch.py  # Uruchamia symulację w Gazebo
├── trailblazerml
├── trailblazer_nav2 # Pakiet nawigacyjny (Nav2)
│   ├── config
│   │   └── nav2_params.yaml
│   ├── launch
│   │   └── navigation_launch.py
│   └── trailblazer_nav2
│       ├── __init__.py
│       ├── __pycache__
│       │   ├── __init__.cpython-310.pyc
│       │   └── ScanDoubleNode.cpython-310.pyc
│       └── ScanDoubleNode.py
└── trailblazer_rviz # Pakiet konfiguracji RViz
    ├── config
    │   ├── nav2.rviz
    │   ├── slam.rviz
    │   └── urdf_model.rviz
    ├── launch
    │   └── rviz.launch.py
    └── trailblazer_rviz
        └── __init__.py

```
### Launching 
#### Gazebo simulation
Not working yet
```shell
ros2 launch trailblazer_gazebo launch_sim.launch.py
```

#### Nawigacja z RVIZ i SLAM

Aby uruchomić autonomiczną nawigację za pomocą pakietu Nav2, wpisz:
```shell
ros2 launch trailblazer_bringup bringup_nav.launch.py
```

#### SLAM z RViz

Aby uruchomić SLAM i obserwować mapowanie w RViz, użyj komendy:
```shell
ros2 launch trailblazer_bringup bringup_slam.launch.py
```

