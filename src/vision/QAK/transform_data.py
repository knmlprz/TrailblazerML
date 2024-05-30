import reading_data as rd
import numpy as np
import open3d as o3d
import cv2
import os

import numpy as np


def process_imu_data(imu_data, initial_position=np.array([0, 0, 0]), initial_velocity=np.array([0, 0, 0])):
    # Założenia:
    # imu_data: lista krotek (accelerometer_reading, gyroscope_reading, timestamp)
    # accelerometer_reading: np.array([ax, ay, az])
    # gyroscope_reading: np.array([gx, gy, gz])
    # timestamp: czas w sekundach

    position = initial_position
    velocity = initial_velocity
    orientation = np.eye(3)  # Macierz jednostkowa 3x3 reprezentująca początkową orientację

    positions = [position.copy()]  # Lista do przechowywania wynikowych pozycji

    last_time = None
    for accelerometer, gyroscope, timestamp in imu_data:
        if last_time is None:
            last_time = timestamp
            continue

        dt = timestamp - last_time
        last_time = timestamp

        # Oblicz prędkości kątowe i aktualizuj orientację (prosty przykład)
        # Można użyć bardziej skomplikowanego filtru jak np. filtr Madgwicka czy filtr Kalmana
        angular_velocity = gyroscope
        orientation += orientation @ skew_symmetric(angular_velocity) * dt  # Przybliżenie Eulera

        # Aktualizacja prędkości i pozycji na podstawie odczytów z akcelerometru
        # Zakładamy, że akcelerometr daje nam przyspieszenie w lokalnym układzie współrzędnych
        acceleration = orientation @ accelerometer  # Przekształć odczyt do globalnego układu współrzędnych
        acceleration -= np.array([0, 0, 9.81])  # Kompensacja przyspieszenia grawitacyjnego

        velocity += acceleration * dt
        position += velocity * dt + 0.5 * acceleration * dt ** 2

        positions.append(position.copy())

    return np.array(positions)


def skew_symmetric(v):
    """Zwraca macierz antysymetryczną dla wektora v."""
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def flie_to_point_imu(imu_path):
    # Odczytaj dane z pliku i przekonwertuje j na
    return points


# Wywołanie funkcji


if __name__ == "__main__":
    base_path = "../../../data/exp5/stairs4"

    poses = sorted(
        [os.path.join(base_path, "absolute_pose", f) for f in os.listdir(os.path.join(base_path, "absolute_pose")) if
         f.endswith('.txt')])

    Imu_data = []
    for pose in poses:
        Imu_data.append(rd.read_pose(pose))

    print(Imu_data)

    positions = process_imu_data(Imu_data)
    print(positions)

    print(poses)
    pose = rd.read_pose(poses[100])
    print(pose)



    rd.visualize_point_clouds_with_trajectory(images, sparse_depths, poses,
                                           "../../../data/exp5/stairs4/K.txt")
