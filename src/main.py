import map2d.translator as m2d

if __name__ == "__main__":
    # Wczytaj przykładową chmurę punktów
    pcd = m2d.generate_random_point_cloud(num_points=1000, value_range=(-1, 1))
    
    # Pozycja z akcelerometru (przykładowe wartości)
    accel_position = (2, 3, 1)

    # Rzutowanie chmury punktów na mapę 2D z wizualizacją
    print(m2d.point_cloud_to_2d_map(pcd, accel_position=accel_position, visualization=True))