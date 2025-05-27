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

# new version
```bash
ros2 launch trailblazer_bringup camera_robot.launch.py 
ros2 launch trailblazer_bringup all.launch.py 
ros2 launch trailblazer_nav2 dual_ekf_navsat.launch.py 
ros2 run trailblazer_gps rtk_gps --ros-args --param port:=/dev/ttyUSB0
sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
ros2 launch trailblazer_nav2 mapviz.launch.py
ros2 topic echo /diff_drive_controller/cmd_vel 
ros2 topic echo /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist 
ros2 run trailblazer_nav2 interactive_waypoint_follower 
ros2 run trailblazer_nav2 logged_waypoint_follower 

```

# actual version
```bash
ros2 topic pub /gps/fix sensor_msgs/msg/NavSatFix '{header: {frame_id: "map"}, latitude: 51.1079, longitude: 17.0385, altitude: 120.0, position_covariance_type: 0}' 

```
```bash
ros2 launch trailblazer_bringup all.launch.py 
ros2 run trailblazer_gps rtk_gps --ros-args --param port:=/dev/ttyUSB0
sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
ros2 launch trailblazer_nav2 mapviz.launch.py
ros2 topic echo /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist 
ros2 run trailblazer_nav2 interactive_waypoint_follower 
```

# Trailblazer Bringup

Paczka `trailblazer_bringup` w projekcie TrailblazerML służy do integracji i uruchamiania wszystkich kluczowych komponentów robota, takich jak nawigacja, sensory (kamery OAK-D, GPS), sterowanie i wizualizacja. Zawiera pliki launch, które łączą moduły jak RTAB-Map, Nav2, GPS i kontrolery w spójny system robota.

## 📚 Spis treści

- [📁 Struktura katalogów](#-struktura-katalogów)
- [⚙️ Używany sprzęt](#️-używany-sprzęt)
- [📄 Opis działania](#-opis-działania)
- [🛠️ Jak używać](#️-jak-używać)
  - [✅ Zbudowanie paczki](#-zbudowanie-paczki)
  - [🚀 Uruchomienie systemu](#-uruchomienie-systemu)
  - [🧩 Integracja z innymi paczkami](#-integracja-z-innymi-paczkami)

## 📁 Struktura katalogów

    src/trailblazer_bringup
    ├── launch              # Pliki startowe (np. all.launch.py, multiple_cam_all.launch.py)
    ├── package.xml         # Zależności, opis, wersja
    └── setup.py            # Plik instalacyjny

## ⚙️ Używany sprzęt

Podczas pracy z paczką `trailblazer_bringup` używamy:
- Kamera OAK-D (np. OAK-D, OAK-D-LITE, OAK-D-PRO-W)
- Moduł GPS (np. LC29H z RTK) podłączony przez port szeregowy (np. /dev/ttyUSB0)
- Robot z kontrolerami ROS 2 (np. trailblazer_controller)

## 📄 Opis działania

Paczka `trailblazer_bringup` integruje różne moduły TrailblazerML w jeden system robota. Plik `all.launch.py` uruchamia:
- Nawigację Nav2 z konfiguracją RGB-D (`trailblazer_rgbd_nav2_params.yaml`).
- RTAB-Map z kamery OAK-D do mapowania i lokalizacji.
- Węzeł GPS (`rtk_gps`) publikujący dane na temat `/gps/fix`.
- Kontrolery robota (`controller.launch.py`) i robot state publisher (`rsp.launch.py`).
- NavSat dla fuzji danych GPS z odometrią.

Plik `multiple_cam_all.launch.py` rozszerza to o obsługę wielu kamer OAK-D (np. OAK-D-LITE), konfigurując je z różnymi przesunięciami (`cam_pos_y`) i integrując z Nav2, RTAB-Map i RViz. Oba pliki wspierają tryb lokalizacji (`localization`) i używają ROS 2 Control.

## 🛠️ Jak używać

### ✅ Zbudowanie paczki

```bash
cd ~/TrailblazerML
colcon build --packages-select trailblazer_bringup
source install/setup.bash
```

### 🚀 Uruchomienie systemu

- Uruchomienie standardowego systemu z jedną kamerą:
  ```bash
  ros2 launch trailblazer_bringup all.launch.py
  ```

- Uruchomienie z wieloma kamerami i wizualizacją:
  ```bash
  ros2 launch trailblazer_bringup multiple_cam_all.launch.py
  ```

### 🧩 Integracja z innymi paczkami

```python
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Integracja z Nav2
nav2_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource('trailblazer_nav2/launch/navigation_launch.py'),
    launch_arguments={'use_sim_time': 'false', 'params_file': 'trailblazer_nav2/config/trailblazer_rgbd_nav2_params.yaml'}
)

# Integracja z RTAB-Map
rtabmap_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource('trailblazer_cloud/launch/depthai.launch.py'),
    launch_arguments={'localization': 'false', 'use_ros2_control': 'true'}
)

# Węzeł GPS
gps_node = Node(
    package='trailblazer_gps',
    executable='rtk_gps',
    name='gps_node',
    parameters=[{'port': '/dev/ttyUSB0'}]
)
```

---
