import open3d as o3d
import numpy as np
import os
import json


def load_config(config_path: str = "./src/utils/sectors_conf.json") -> dict:
    with open(config_path, "r") as f:
        config = json.load(f)
    return config


def make_sectors(path: str = "./") -> None:
    """
    Generates a matrix representing sectors based on given array size and sector size.

    The generated matrix has the structure:
    [ [x, x + sector_size, y - sector_size, y], ... ]

    The file is saved to the specified path in the .npy format.

    :param path: File path where the output matrix should be saved.
    :return: None
    """
    config = load_config()
    sector_size = config["sector_size"]
    array_size = config["array_size"]
    x = np.arange(array_size) * sector_size
    y = np.arange(array_size) * sector_size + sector_size
    xv, yv = np.meshgrid(x, y)

    combined_matrix = np.zeros((array_size, array_size, 2, 2))
    combined_matrix[..., 0, 0] = xv
    combined_matrix[..., 0, 1] = xv + sector_size
    combined_matrix[..., 1, 0] = yv - sector_size
    combined_matrix[..., 1, 1] = yv
    path = os.path.join(path, "sectors.npy")
    np.save(path, combined_matrix)


def assignment_to_sectors(
    pcd: o3d.geometry.PointCloud, path: str = "./src/vision/oak/sectors.npy"
):
    """
    Assigns points from a point cloud to sectors and calculates the center of mass for each sector.

    The function takes a 3D point cloud, divides the space into sectors of a fixed size, and computes the
    center of mass for the points in each sector. The result is a 2D matrix where each cell represents
    a sector and contains the center of mass value for that sector.

    Parameters:
    pcd (o3d.geometry.PointCloud): The input point cloud containing 3D points.
    path (str): File path where the sectors numpy array is loaded from. Default is './src/vision/oak/sectors.npy'.

    Returns (np.ndarray): A 2D numpy array where each cell contains the center of mass value for the corresponding sector.
                The array size is determined by the maximum sector indices found in the point cloud.
    """
    config = load_config()
    sector_size = config["sector_size"]

    try:
        sectors = np.load(path)
    except FileNotFoundError:
        print(f"sectors.npy not found at {path}. Generating new file.")
        make_sectors(path=os.path.dirname(path))
        sectors = np.load(path)

    points = np.asarray(pcd.points)

    # Calculate x and z indices for each point
    x_indices = (points[:, 2] // sector_size).astype(int)
    z_indices = (points[:, 0] // sector_size).astype(int)

    # Filter points to only include those within valid sector ranges
    valid_mask = (
        (x_indices >= 0)
        & (x_indices < sectors.shape[0])
        & (z_indices >= 0)
        & (z_indices < sectors.shape[1])
    )
    x_indices = x_indices[valid_mask]
    z_indices = z_indices[valid_mask]
    valid_points = points[valid_mask]

    # Find unique sectors
    unique_sectors = np.unique(np.vstack((x_indices, z_indices)).T, axis=0)

    # Calculate center of mass for each sector
    centers_of_masses = {}
    y_axis = 1
    for sector in unique_sectors:
        sector_mask = (x_indices == sector[0]) & (z_indices == sector[1])
        sector_points = valid_points[
            sector_mask, y_axis
        ]  # Use y values for the center of mass calculation
        masses = np.ones_like(sector_points)
        center_of_mass = np.sum(sector_points * masses) / np.sum(masses)
        centers_of_masses[(sector[0], sector[1])] = center_of_mass

    # Extract keys and values from the centers_of_masses dictionary
    sectors = np.array(list(centers_of_masses.keys()))

    # Determine the size of the resulting matrix
    max_index = np.max(sectors)
    results = np.full((max_index + 1, max_index + 1), np.nan)

    # Assign center of mass values to the result matrix
    for (x_idx, z_idx), center_mass in centers_of_masses.items():
        results[x_idx, z_idx] = center_mass

    return results
