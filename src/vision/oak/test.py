import numpy as np
from pykalman import KalmanFilter
import matplotlib.pyplot as plt
import open3d as o3d
from matplotlib.animation import FuncAnimation
#
# class RealTimePlotter:
#     def __init__(self):
#         self.fig, self.axs = plt.subplots(9, 1, figsize=(10, 20))  # Creating 9 subplots
#         self.time_stamps = []
#         self.data = {i: [] for i in range(9)}  # Dictionary to store data for each plot
#
#         self.init_plot()
#
#     def init_plot(self):
#         self.lines = []
#         for i, ax in enumerate(self.axs):
#             line, = ax.plot([], [], lw=2)
#             self.lines.append(line)
#             ax.set_xlim(0, 5)
#             ax.set_ylim(-1, 1)
#             title = ['Accel X', 'Accel Y', 'Accel Z', 'Gyro X', 'Gyro Y', 'Gyro Z', 'Orient I', 'Orient J', 'Orient K']
#             ax.set_title(title[i])
#
#     def update(self, delta_t, accel_data, gyro_data, orientation_data):
#         current_time = self.time_stamps[-1] + delta_t if self.time_stamps else 0
#         self.time_stamps.append(current_time)
#
#         # Update data for each plot
#         for i, value in enumerate(accel_data + gyro_data + orientation_data):
#             self.data[i].append(value)
#
#         # Update each plot
#         for i, line in enumerate(self.lines):
#             line.set_data(self.time_stamps, self.data[i])
#             line.axes.axis([current_time - 5, current_time + 5, -1, 1])
#
#         plt.draw()
#
#     def run(self):
#         plt.show()
#

class ImuTracker:
    def __init__(self, visualize: bool = True):
        self.visualize = visualize
        # self.n_dim_state = 10
        # self.kalman_filter = KalmanFilter(n_dim_state=self.n_dim_state, n_dim_obs=7)
        #
        # # Macierze przejścia i obserwacji
        # self.kalman_filter.transition_matrices = np.eye(self.n_dim_state)
        # self.kalman_filter.observation_matrices = np.hstack([np.eye(6, self.n_dim_state), np.zeros((6, 4))])[:7, :10]
        #
        # # Dodanie quaternionu do macierzy obserwacji
        # self.kalman_filter.observation_matrices = np.vstack(
        #     [self.kalman_filter.observation_matrices, np.array([0, 0, 0, 0, 0, 0, 1, 1, 1, 1])])
        #
        # # Kovariancja procesu i obserwacji
        # self.kalman_filter.transition_covariance = np.eye(self.n_dim_state) * 0.01
        # self.kalman_filter.observation_covariance = np.eye(7) * 0.1
        #
        # # Wektory stanu początkowego i kowariancji
        # self.kalman_filter.initial_state_mean = np.zeros(self.n_dim_state)
        # self.kalman_filter.initial_state_covariance = np.eye(self.n_dim_state) * 1.0
        # self.current_state_mean = self.kalman_filter.initial_state_mean
        # self.current_state_covariance = self.kalman_filter.initial_state_covariance
        self.list_accel_data0= []
        self.list_accel_data1 = []
        self.list_accel_data2 = []
        self.list_accel_data3 =[]



        # Tworzenie ramki współrzędnych
        self.mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])

        # Inicjalizacja wizualizatora
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()

        # Dodanie siatki do wizualizatora
        self.vis.add_geometry(self.mesh)

    def update(self, accel_data, gyro_data, rotation_vector, delta_t):
        pose = self.quaternion_rotation_matrix(rotation_vector)
        print(pose)
        self.mesh.rotate(pose)
        self.vis.update_geometry(self.mesh)
        self.vis.poll_events()
        self.vis.update_renderer()
        # if self.visualize:
        #     self.list_accel_data0.append(rotation_vector[0])
        #     self.list_accel_data1.append(rotation_vector[1])
        #     self.list_accel_data2.append(rotation_vector[2])
        #     self.list_accel_data3.append(rotation_vector[3])
        #
        #     plt.plot(self.list_accel_data0)
        #     plt.plot(self.list_accel_data1)
        #     plt.plot(self.list_accel_data2)
        #     plt.plot(self.list_accel_data3)
        #     plt.draw()
        #     plt.pause(0.1)
        #     plt.clf()



        # Przygotowanie wektora obserwacji
        observation = np.hstack([accel_data, gyro_data, rotation_vector])

        # # Aktualizacja filtru Kalmana
        # self.current_state_mean, self.current_state_covariance = self.kalman_filter.filter_update(
        #     self.current_state_mean,
        #     self.current_state_covariance,
        #     observation=observation
        # )

        # Zaktualizowane przewidywania
        # position = self.current_state_mean[:3]
        # velocity = self.current_state_mean[3:6]
        # orientation = self.current_state_mean[6:10]
        #
        # return position, velocity, orientation

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
        q0 = Q[2] # 0 1 2 3
        q1 = Q[2] # 1 2 3 0
        q2 = Q[1] # 2 3 0 1
        q3 = Q[0] # 3 0 1 2

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

