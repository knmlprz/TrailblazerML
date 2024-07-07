import numpy as np
from pykalman import KalmanFilter
import matplotlib.pyplot as plt
import open3d as o3d

class ImuTracker:
    def __init__(self, visualize: bool = True):
        self.current_point = np.array([0, 0, 0])
        self.current_velocity = np.array([0, 0, 0])

    def update_wo_kalman(self, accel_data: (float,float,float), delta_t):
        accel_data = np.array(accel_data)*1000

        delta_velocity = accel_data * delta_t
        self.current_velocity = self.current_velocity + delta_velocity
        new_position = self.current_point + self.current_velocity * delta_t + 0.5* accel_data*delta_t**2
        self.current_point = new_position


        return np.array(new_position)

    def update(self, accel_data, rotation_vector, delta_t ):
        new_point = self.update_wo_kalman(accel_data, delta_t)
        new_point = new_point.reshape(3, 1)
        bottom_row = np.array([[0, 0, 0, 1]])
        rotation_matrix = self.quaternion_rotation_matrix(rotation_vector)
        pose = np.vstack([
            np.hstack([rotation_matrix, new_point]),
            bottom_row
        ])
        return pose

    def quaternion_rotation_matrix(self, Q):
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
        q0 = Q[0] # 0 1 2 3
        q1 = Q[1] # 1 2 3 0
        q2 = Q[2] # 2 3 0 1
        q3 = Q[3] # 3 0 1 2

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

    def create_pose_matrix(self, position, quaternion):
        rotation_matrix = self.quaternion_to_rotation_matrix(quaternion)
        pose_matrix = np.eye(4)
        pose_matrix[:3, :3] = rotation_matrix
        pose_matrix[:3, 3] = position
        return pose_matrix