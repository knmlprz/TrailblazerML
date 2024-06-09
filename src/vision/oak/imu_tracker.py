import numpy as np


class ImuTracker:
    def __init__(self):
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.eye(3)
        self.initialized = False

    def quaternion_to_rotation_matrix(self, q: list) -> np.array:
        """ Convert a quaternion into a rotation matrix.
        Args:
            q (list): The quaternion to convert.
        Returns:
            np.array: The rotation matrix 3x3.

        """
        w, x, y, z = q
        return np.array([
            [1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x ** 2 - 2 * y ** 2]
        ])

    def update(self, accel_data: list, rotation_vector: list, delta_t) -> np.array:
        """ Update the IMU pose estimation.
        Args:
            accel_data (list): The acceleration data of .
            rotation_vector (list): The rotation vector.
            delta_t (float): The time delta. from the last update.
        Returns:
            np.array: The transformation matrix 4x4.
        """
        # Convert the rotation vector to a rotation matrix
        rotation_matrix = self.quaternion_to_rotation_matrix(rotation_vector)

        # If this is the first update, initialize the orientation
        if not self.initialized:
            self.orientation = rotation_matrix
            self.initialized = True
            return np.eye(4)  # Return the identity matrix as the initial pose

        # Update velocity and position
        accel_world = np.dot(self.orientation, accel_data)  # Convert acceleration to world coordinates
        self.velocity += accel_world * delta_t
        self.position += self.velocity * delta_t + 0.5 * accel_world * delta_t ** 2

        # Update orientation
        self.orientation = np.dot(rotation_matrix, self.orientation)

        # Construct the 4x4 pose matrix
        pose_matrix = np.eye(4)
        pose_matrix[:3, :3] = self.orientation
        pose_matrix[:3, 3] = self.position

        return pose_matrix
