# Trailblazer nav2
Paczka trailblazer_nav2 odpowiada za autonomiczną nawigację robota Trailblazer z wykorzystaniem systemu Nav2 w ROS 2.
Zawiera konfigurację lokalizacji, nawigacji. Jej celem jest umożliwienie robotowi samodzielnego przemieszczania się po znanym terenie oraz łatwe dostosowanie ustawień do własnych map i tras.

TODO:
- [ ] Dodać gotowe przykłady mapy (maps/)
- [ ] Opisać jak dodać własne punkty GPS do jazdy

## 📚 Spis treści

- [📁 Struktura katalogów](#-struktura-katalogów)
- [🗺️ GPS i GNNS](#-gps-i-gnss)
    - [❔ Co to jest GPS/GNSS?](#-co-to-jest-gpsgnss)
    - [❔ Co to UTM?](#-co-to-utm)
- [🧭 Lokalizacja robota](#-lokalizacja-robota)
    - [📦 Konfiguracja lokalizacji](#-konfiguracja-lokalizacji)
- [🚗 Autonomiczna jazda](#-autonomiczna-jazda)
    - [⚙️ Dostosowanie ustawień AMCL](#️-dostosowanie-ustawień-amcl)
    - [⚙️ Dostosowanie ustawień bt_navigator](#️-dostosowanie-ustawień-bt_navigator)
    - [⚙️ Dostosowanie ustawień controller_server](#️-dostosowanie-ustawień-controller_server)
    - [⚙️ Dostosowanie ustawień local_costmap](#️-dostosowanie-ustawień-local_costmap)
    - [⚙️ Dostosowanie ustawień global_costmap](#️-dostosowanie-ustawień-global_costmap)
    - [⚙️ Dostosowanie ustawień map_server](#️-dostosowanie-ustawień-map_server)
    - [⚙️ Dostosowanie ustawień planner_server](#️-dostosowanie-ustawień-planner_server)
    - [⚙️ Dostosowanie ustawień smoother_server](#️-dostosowanie-ustawień-smoother_server)
    - [⚙️ Dostosowanie ustawień behavior_server](#️-dostosowanie-ustawień-behavior_server)
    - [⚙️ Dostosowanie ustawień waypoint_follower](#️-dostosowanie-ustawień-waypoint_follower)
    - [⚙️ Dostosowanie ustawień velocity_smoother](#️-dostosowanie-ustawień-velocity_smoother)
- [👣 Co to base footprint?](#-co-to-base-footprint?)
- [🛠️ Jak używać](#️-jak-używać)
- [🔗 Linki](#-linki)

## 📁 Struktura katalogów
    src/trailblazer_nav2
    ├── config              # Pliki konfiguracyjne
    ├── launch              # Pliki startowe
    ├── maps                # Mapy
    ├── package.xml         # Zależności, opis, wersja
    ├── setup.py            # Plik instalacyjny
    └── trailblazer_nav2    # Moduł paczki

## 🗺️ GPS i GNNS
### ❔ Co to jest GPS/GNSS?
GPS/GNSS to systemy, które używają satelit, by określić Twoje położenie (szerokość, długość i wysokość geograficzną).Standardowo pozycję podaje się w układzie ***WGS84*** – to taki "globalny" system XYZ, którego środek jest w środku Ziemi.
<div align="center">
  <img src="images/image.png" width="500" height="500">
</div>

Nawigowanie robota w takim systemie byłoby niewygodne — lepiej mówić mu "jedź 100 metrów na północ", a nie przesuwaj się o 0.001 stopnia szerokości geograficznej.

### ❔ Co to UTM?
Aby rozwiązać ten problem, używa się systemu UTM (Universal Transverse Mercator).
UTM dzieli Ziemię na strefy i tworzy lokalne układy współrzędnych w metrach, co znacznie ułatwia nawigację dla robotów bo zamiast globalnych współrzędnych mamy lokalne X i Y w metrach.
<div align="center">
  <img src="images/image-1.png" width="500" height="600">
</div>

## 🧭 Lokalizacja robota
Robot musi wiedzieć gdzie jest i w którą stronę patrzy. GPS daje tylko pozycję, ale nie orientację (czyli "w którą stronę przód robota jest skierowany"). Rozwiązaniem tego jest zastosowanie IMU (Inertial Measurement Unit), które mierzy przyspieszenia i obroty. W skrócie:
- GPS mówi „gdzie jestem?”,
- IMU mówi „w którą stronę jestem obrócony?”.

### 📦 Konfiguracja lokalizacji
Do dokładniejszego połączenia tych danych (IMU, enkodery i GPS) używamy `filtra EKF` (Extended Kalman Filter) oraz  rzetwarzaniu danych GNSS za pomocą `navsat_transform` z pakietu [robot_localization](https://docs.ros.org/en/melodic/api/robot_localization/html/index.html).

1) `navsat_transform_node` - Przelicza dane z GPS (dane geograficzne: długość/szerokość/ wysokość) na układ współrzędnych X, Y, Z 
2) `ekf_filter_node_odom` (mały EKF) - Łączy odometrię z IMU lokalnie, żeby mieć dokładne śledzenie ruchu robota w krótkiej skali (czyli bez użycia GPS)
3) `ekf_filter_node_map` (duży EKF) - Łączy wyniki małego EKF + pozycję GPS + IMU, żeby wiedzieć gdzie robot jest globalnie na mapie.

> [!NOTE]
> Poprawny łańcuch transformacji wygląda:
> 
> map (z ekf_filter_node_map) ➡️ odom (z ekf_filter_node_odom) ➡️ base_link

1) `ekf_filter_node_odom` oraz `ekf_filter_node_map`
```yaml
kf_filter_node_odom:
  ros__parameters:
    frequency: 30.0                     # Częstotliwość aktualizacji
    two_d_mode: true                    # Ignoruje ruch w osi "Z" 2D tryb (dla robotów po ziemi)
    print_diagnostics: false            # Wypisywanie informagi diagnostycznych np. WARNING
    debug: false                        # Możliwość tworzenia plików log
    publish_tf: true                    # Publikuj transformację odom → base_link

    map_frame: map                      # Nazwa frame mapy
    odom_frame: odom                    # Nazwa frame odometrii
    base_link_frame: base_link          # Nazwa frame base_linku
    world_frame: odom                   # Nazwa frame świata lokalnego
```
Opis źródła danych do EKF definiuje się za pomocą listy w której kolejne wartości (bool) odpowiadają za:

    [   x_pos   , y_pos    , z_pos,
        roll    , pitch    , yaw,
        x_vel   , y_vel    , z_vel,
        roll_vel, pitch_vel, yaw_vel,
        x_accel , y_accel  , z_accel    ]

```yaml
odom0_queue_size: 10                            # Kolejka wiadomości oczekujących na przetworzenie
odom0_differential: false                       # Nie traktuj odometrii jako różnicowej tylko jako absolutne pomiary
odom0_relative: false                           # Nie przekształcaj danych na relatywne

imu0_differential: true                         # Czyli traktujesz zmianę kąta yaw jako delta (dobre dla tanich IMU)
imu0_relative: false                            # Dane nie są przekształcane na względne
imu0_queue_size: 10
imu0_remove_gravitational_acceleration: true    # Usuwa wpływ grawitacji z pomiarów przyspieszenia (normalne dla IMU)

use_control: false                              # Nie używa żadnych danych o sterowaniu
process_noise_covariance: [...]                 # Macierz określająca jak bardzo EKF ufa swoim przewidywaniom
```

2) `navsat_transform`
```yaml
navsat_transform:
  ros__parameters:
    frequency: 30.0                     # Częstotliwość aktualizacji
    delay: 3.0                          # Opóźnienie działania w sekundach
    magnetic_declination_radians: 0.0   # Korekta na lokalną deklinację magnetyczną 
    yaw_offset: 0.0                     # Jeśli twoje "przód" GPS/IMU różni się od fizycznego przodu robota, można tu dodać korektę w radianach
    zero_altitude: true                 # Ignoruje wysokość (Z) z GPS 
    broadcast_utm_transform: true       # Publikuje transformację pomiędzy mapą (np. UTM) a lokalną mapą robota
    publish_filtered_gps: true          # Publikuje filtrowane dane GPS do odczytu.
    use_odometry_yaw: true              # Używa yaw (orientacji) z odometrii, a nie z GPS/IMU.
    wait_for_datum: false               # Nie czeka na manualne ustawienie punktu odniesienia, tylko automatycznie używa pierwszego GPS-a.
```


## 🚗 Autonomiczna jazda
Celem autonomicznej jazdy jest umożliwienie robotowi Trailblazer samodzielnego przemieszczania się po zaplanowanej trasie, bazując na punktach GPS lub na wyznaczonej ścieżce na mapie.

<div align="center">
  <img src="images/image-3.png" width="800" height="700">
</div>

### ⚙️ Dostosowanie ustawień AMCL
AMCL to algorytm lokalizacji robota na mapie.
```yaml
amcl:
  ros__parameters:
    base_frame_id: "base_link"
    global_frame_id: "map"
    odom_frame_id: "odom"
    scan_topic: scan
```

### ⚙️ Dostosowanie ustawień bt_navigator
BT Navigator zarządza misją robota przy użyciu tzw. Behavior Trees (BT) – drzew zachowań.
Dzięki temu robot wie np.:
- jak jechać do celu,
- jak unikać przeszkód,
- co robić gdy coś pójdzie nie tak.
```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
```

### ⚙️ Dostosowanie ustawień controller_server
Controller Server odpowiada za sterowanie robotem w czasie rzeczywistym po zaplanowanej trasie.
```yaml
controller_server:
  ros__parameters:
    min_x_velocity_threshold: 0.2       # Minimalna dopuszczalna prędkość w osi X
    min_y_velocity_threshold: 0.2       # Minimalna dopuszczalna prędkość w osi Y
    min_theta_velocity_threshold: 0.2   # Minimalna dopuszczalna prędkość w osi obrotu
  progress_checker:
    required_movement_radius: 0.8       # Minimalne przesunięcie robota (w metrach) w określonym czasie
  general_goal_checker:
    xy_goal_tolerance: 0.6              # Tolerancja w XY (metry) kiedy uznajemy, że osiągnęliśmy cel
    yaw_goal_tolerance: 1.57            # Tolerancja kąta obrotu (radiany)
  FollowPath:
    min_vel_x: 0.0                      # Minimalna dopuszczalna prędkość robota w osi X
    max_vel_x: 0.7                      # Maksymalna prędkość jazdy robota w osi X m/s.
    max_vel_theta: 0.5                  # Maksymalne prędkość obrotu robota
    min_speed_xy: 0.1                   # Minimalna prędkość wypadkowa w płaszczyźnie XY
    max_speed_xy: 1.0                   # Maksymalna prędkość wypadkowa w płaszczyźnie XY
    min_speed_theta: 0.2                #	Minimalna prędkość obrotowa
    linear_granularity: 0.1             # Precyzja "kroków" w przestrzeni, im mniejsza wartość, tym dokładniejsze sterowanie
    angular_granularity: 0.1            # Precyzja "kroków" w przestrzeni obrotowej
    transform_tolerance: 0.2            # Tolerancja czasowa przy odczycie transformacji
    xy_goal_tolerance: 0.6              # Tolerancja osiągnięcia celu w XY (metry)
```

### ⚙️ Dostosowanie ustawień local_costmap
```yaml
local_costmap:
  ros__parameters:
    width: 4
    height: 4
```

### ⚙️ Dostosowanie ustawień global_costmap
```yaml
global_costmap:
  ros__parameters:
```

### ⚙️ Dostosowanie ustawień map_server
```yaml
map_server:
  ros__parameters:
```

### ⚙️ Dostosowanie ustawień planner_server
```yaml
planner_server:
  ros__parameters:
```

### ⚙️ Dostosowanie ustawień smoother_server
```yaml
smoother_server:
  ros__parameters:
```

### ⚙️ Dostosowanie ustawień behavior_server
```yaml
behavior_server:
  ros__parameters:
```

### ⚙️ Dostosowanie ustawień waypoint_follower
```yaml
waypoint_follower:
  ros__parameters:
```

### ⚙️ Dostosowanie ustawień velocity_smoother
```yaml
velocity_smoother:
  ros__parameters:
```

## 👣 Co to base footprint?
Base footprint to dwuwymiarowy (2D) obrys kształtu robota, rzutowany na płaszczyznę podłoża. W systemie nawigacji Nav2 w ROS 2, obrys ten służy głównie do detekcji kolizji podczas planowania tras i omijania przeszkód.
<div align="center">
  <img src="images/image-2.png" width="700" height="500">
</div>

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 1.3
      footprint: "[[0.7, 0.65], [0.7, -0.65], [-0.7, -0.65], [-0.7, 0.65]]"

global_costmap:
  global_costmap:
    ros__parameters:
      robot_radius: 1.0
```

> [!NOTE]
> Jeśli w konfiguracji podamy oba parametry (`footprint` i `robot_radius`) Nav2 skorzysta z dokładniejszego kształtu wielokąta (footprint).


## 🛠️ Jak używać
### ✅ Zbudowanie paczki
```bash
cd ~/TrailblazerML
colcon build --packages-select trailblazer_nav2
source install/setup.bash
```
### 🚀 Uruchomienie symulacji samodzielnie
```bash

```

### 🧩 Integracja z innymi paczkami
```python

```

## 🔗 Linki
- https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html
- https://github.com/ros-navigation/navigation2_tutorials/tree/rolling/nav2_gps_waypoint_follower_demo
- https://docs.nav2.org/tutorials/docs/navigation2_dynamic_point_following.html
- https://docs.nav2.org/setup_guides/footprint/setup_footprint.html
- https://automaticaddison.com/ros-2-navigation-tuning-guide-nav2/#local_costmap
