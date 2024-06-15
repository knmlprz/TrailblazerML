import reading_data as rd
import numpy as np
import open3d as o3d
import cv2
import os
import pandas as pd
from numpy import save


def make_sectors(array_size, sector_size):
    x = np.arange(array_size) * sector_size
    y = np.arange(array_size) * sector_size + sector_size  # Start y values from 0.01
    xv, yv = np.meshgrid(x, y)

    # Combine x and y into pairs, making each element contain the corresponding x and y values
    combined_matrix = np.zeros((array_size, array_size, 2, 2))
    combined_matrix[..., 0, 0] = xv
    combined_matrix[..., 0, 1] = xv + sector_size
    combined_matrix[..., 1, 0] = yv - sector_size
    combined_matrix[..., 1, 1] = yv

    return combined_matrix


def assignment_to_sectors(sectors, pcd):
    # assign 3d points to 2d sectors
    pass


matrix = make_sectors(array_size=5000, sector_size=0.01)
save('data.npy', matrix)
