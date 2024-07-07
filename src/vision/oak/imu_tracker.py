import numpy as np


class ImuTracker:
    def __init__(self, visualize: bool = True):
        self.current_point = np.array([0, 0, 0])
        self.current_velocity = np.array([0, 0, 0])
        self.pose_matrix = np.eye(4)

    def update_position(self, accel_data: (float, float, float), delta_t):
        """
        Update the position of the camera based on the acceleration data and the time passed.
        Args:
            accel_data:
            delta_t:

        Returns:

        """
        accel_data = np.array([accel_data[0], accel_data[1], accel_data[2]]) * 1000
        delta_velocity = accel_data * delta_t
        self.current_velocity = self.current_velocity + delta_velocity
        new_position = self.current_point + self.current_velocity * delta_t + 0.5 * accel_data * delta_t ** 2
        self.current_point = np.array([new_position[0], 0, new_position[2]])
        return self.current_point

    def update(self, accel_data: (float, float, float), rotation_vector: (float, float, float),
               delta_t: float) -> np.ndarray:
        """
        Update the position of the camera based on the acceleration data, rotation vector and the time passed.
        Args:
            accel_data:
            rotation_vector:
            delta_t:

        Returns:

        """
        new_position = self.update_position(accel_data, delta_t)
        rotation_matrix = self.quaternion_rotation_matrix(rotation_vector)
        self.pose_matrix[:3, :3] = rotation_matrix
        self.pose_matrix[:3, 3] = new_position
        return self.pose_matrix

    def quaternion_rotation_matrix(self, Q: (float, float, float, float)) -> np.ndarray:
        """
        Covert a quaternion into a full three-dimensional rotation matrix.

        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3)

        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix.
                 This rotation matrix converts a point in the local reference
                 frame to a point in the global reference frame.
        """
        # Extract the values from Q
        q0 = Q[0]  # 0 1 2 3
        q1 = Q[1]  # 1 2 3 0
        q2 = Q[2]  # 2 3 0 1
        q3 = Q[3]  # 3 0 1 2

        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)

        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)

        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1

        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                               [r10, r11, r12],
                               [r20, r21, r22]])

        return rot_matrix
