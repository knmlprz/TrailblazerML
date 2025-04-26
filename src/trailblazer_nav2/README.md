# Trailblazer nav2
Paczka trailblazer_nav2 odpowiada za autonomiczną nawigację robota Trailblazer z wykorzystaniem systemu Nav2 w ROS 2.
Zawiera konfigurację lokalizacji, nawigacji. Jej celem jest umożliwienie robotowi samodzielnego przemieszczania się po znanym terenie oraz łatwe dostosowanie ustawień do własnych map i tras.

TODO:
- [ ] Dodać gotowe przykłady mapy (maps/)
- [ ] Opisać jak dodać własne punkty GPS do jazdy

## 📚 Spis treści

- [📁 Struktura katalogów](#-struktura-katalogów)
- [🗺️ GPS i GNNS](#-gps-i-gnss)
- [🧭 Lokalizacja robota](#-lokalizacja-robota)
- [🚗 Autonomiczna jazda](#-autonomiczna-jazda)
- [👣 Co to base footprint?](#-co-to-base-footprint?)
- [🛠️ Jak używać](#️-jak-używać)

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
> map (z ekf_filter_node_map) ➡️ odom (z ekf_filter_node_odom) ➡️ base_footprint ➡️ base_link

1) `ekf_filter_node_odom` oraz `ekf_filter_node_map`
```yaml
kf_filter_node_odom:
  ros__parameters:
    frequency: 30.0                     # Częstotliwość aktualizacji
    two_d_mode: true                    # Ignoruje ruch w osi "Z" 2D tryb (dla robotów po ziemi)
    print_diagnostics: false            # Wypisywanie informagi diagnostycznych np. WARNING
    debug: false                        # Możliwość tworzenia plików log
    publish_tf: true                    # Publikuj transformację odom → base_footprint

    map_frame: map                      # Nazwa frame mapy
    odom_frame: odom                    # Nazwa frame odometrii
    base_link_frame: base_footprint     # Nazwa frame base_linku
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

## 👣 Co to base footprint?

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