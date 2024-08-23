# Uruchomienie ROS 2 w Docker na x86

## Wymagania wstępne
- **Docker**: Zainstaluj Docker na swoim systemie [Instalacja Docker](https://docs.docker.com/engine/install/).
- **Podstawowa znajomość ROS 2 i Dockera**.

## Korzystanie z oficjalnych obrazów ROS 2
1. **Pobierz obraz ROS 2 Humble**:
   ```shell
   docker pull osrf/ros:humble-desktop
   ```
2. Uruchom kontener z ROS 2:
   ```shell
   docker run -it --name ros2_humble osrf/ros:humble-desktop bash
   ```
3. Uruchomienie węzłów ROS 2:

- Talker:
   ```shell
   ros2 run demo_nodes_cpp talker
   ```
- Listener:
   ```shell
   ros2 run demo_nodes_cpp listener
   ```