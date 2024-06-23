import open3d as o3d
import numpy as np


def make_sectors(array_size: int, sector_size: float) -> None:
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

    np.save('data.npy', combined_matrix)


def assignment_to_sectors(pcd: o3d.geometry.PointCloud, sector_size: float = 0.01):
    """
    Assigns points from a point cloud to sectors and calculates the center of mass for each sector.

    The function takes a 3D point cloud, divides the space into sectors of a fixed size, and computes the
    center of mass for the points in each sector. The result is a 2D matrix where each cell represents
    a sector and contains the center of mass value for that sector.

    Parameters:
    pcd (o3d.geometry.PointCloud): The input point cloud containing 3D points.

    Returns: (np.ndarray) A 2D numpy array where each cell contains the center of mass value for the corresponding sector.
                The array size is determined by the maximum sector indices found in the point cloud.
    """
    sectors = np.load('./src/vision/oak/data.npy')
    points = np.asarray(pcd.points)

    # Calculate x and z indices for each point
    x_indices = (points[:, 2] // sector_size).astype(int)
    z_indices = (points[:, 0] // sector_size).astype(int)

    # Filter points to only include those within valid sector ranges
    valid_mask = (x_indices >= 0) & (x_indices < sectors.shape[0]) & (z_indices >= 0) & (z_indices < sectors.shape[1])
    x_indices = x_indices[valid_mask]
    z_indices = z_indices[valid_mask]
    valid_points = points[valid_mask]

    # Find unique sectors
    unique_sectors = np.unique(np.vstack((x_indices, z_indices)).T, axis=0)

    # Calculate center of mass for each sector
    centers_of_masses = {}
    for sector in unique_sectors:
        sector_mask = (x_indices == sector[0]) & (z_indices == sector[1])
        sector_points = valid_points[sector_mask, 1]  # Use y values for the center of mass calculation
        masses = np.ones_like(sector_points)
        center_of_mass = np.sum(sector_points * masses) / np.sum(masses)
        centers_of_masses[(sector[0], sector[1])] = center_of_mass

    # Extract keys and values from the centers_of_masses dictionary
    sectors = np.array(list(centers_of_masses.keys()))

    # Determine the size of the resulting matrix
    max_index = np.max(sectors)
    wyniki = np.full((max_index + 1, max_index + 1), np.nan)

    # Assign center of mass values to the result matrix
    for (x_idx, z_idx), center_mass in centers_of_masses.items():
        wyniki[x_idx, z_idx] = center_mass

    return wyniki


# Tworzenie pliku sektorów
make_sectors(5000, 100)
