import numpy as np
from pykalman import KalmanFilter


class ImuTracker:
    def __init__(self):
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.eye(3)
        self.initialized = False
        self.global_transform = np.eye(4)  # Globalna macierz transformacji

        n_dim_state = 9
        self.kf = KalmanFilter(n_dim_state=n_dim_state, n_dim_obs=3)

        self.kf.transition_matrices = np.eye(n_dim_state)
        self.kf.observation_matrices = np.eye(3, n_dim_state)
        self.kf.transition_covariance = np.eye(n_dim_state) * 0.01
        self.kf.observation_covariance = np.eye(3) * 0.1

        self.current_state_mean = np.zeros(n_dim_state)
        self.current_state_covariance = np.eye(n_dim_state) * 1.0

    def quaternion_to_rotation_matrix(self, q):
        #TODO visualisation
        w, x, y, z = q
        return np.array([
            [1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x ** 2 - 2 * y ** 2]
        ])

    def update(self, accel_data, rotation_vector, delta_t):
        if not self.initialized:
            self.orientation = self.quaternion_to_rotation_matrix(rotation_vector)
            self.initialized = True
            self.current_state_mean[:3] = self.position
            self.current_state_mean[3:6] = self.velocity
            self.current_state_mean[6:9] = accel_data
            return self.global_transform  # Początkowa macierz jednostkowa

        self.current_state_mean, self.current_state_covariance = self.kf.filter_update(
            self.current_state_mean,
            self.current_state_covariance,
            observation=accel_data
        )

        self.position = self.current_state_mean[:3]
        self.velocity = self.current_state_mean[3:6]
        new_orientation = np.dot(self.quaternion_to_rotation_matrix(rotation_vector), self.orientation)

        # Budowa lokalnej macierzy transformacji
        local_transform = np.eye(4)
        local_transform[:3, :3] = new_orientation
        local_transform[:3, 3] = self.position - self.global_transform[:3, 3]  # Oblicz przesunięcie względem poprzedniej pozycji

        # Aktualizacja globalnej transformacji
        self.global_transform = np.dot(self.global_transform, local_transform)
        self.orientation = new_orientation

        return self.global_transform
