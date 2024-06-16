import open3d as o3d
import cv2
import os
import pandas as pd
from numpy import save
import numpy as np


def make_sectors(array_size: int, sector_size: float) -> np.ndarray:
    """
    Generates a matrix representing sectors based on given array size and sector size.

    [ x, x + sector_size
     y - sector_size, y ]

    :param array_size: Size of each input array.
    :param sector_size: Size of each sector in the matrix.
    :return: numpy array representing the sectors.
    """
    x = np.arange(array_size) * sector_size
    y = np.arange(array_size) * sector_size + sector_size
    xv, yv = np.meshgrid(x, y)

    combined_matrix = np.zeros((array_size, array_size, 2, 2))
    combined_matrix[..., 0, 0] = xv
    combined_matrix[..., 0, 1] = xv + sector_size
    combined_matrix[..., 1, 0] = yv - sector_size
    combined_matrix[..., 1, 1] = yv

    return combined_matrix


def assignment_to_sectors(pcd):
    sector_size = 0.01
    sectors = np.load('/home/filip/PycharmProjects/TrailblazerML/src/vision/oak/data.npy')
    points = np.asarray(pcd.points)

    x_indices = (points[:, 0] // sector_size).astype(int)
    z_indices = (points[:, 2] // sector_size).astype(int)

    valid_mask = (x_indices >= 0) & (x_indices < sectors.shape[0]) & (z_indices >= 0) & (z_indices < sectors.shape[1])

    x_indices = x_indices[valid_mask]
    z_indices = z_indices[valid_mask]
    valid_points = points[valid_mask]

    sector_center_x = (sectors[x_indices, z_indices, 0, 0] + sectors[x_indices, z_indices, 0, 1]) / 2
    sector_center_z = (sectors[x_indices, z_indices, 1, 0] + sectors[x_indices, z_indices, 1, 1]) / 2

    new_points = np.zeros_like(valid_points)
    new_points[:, 0] = sector_center_x
    new_points[:, 1] = valid_points[:, 1]
    new_points[:, 2] = sector_center_z

    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(new_points)

    return new_pcd


