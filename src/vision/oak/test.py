import cv2
import depthai as dai
import time
import numpy as np
import open3d as o3d


def quaternion_to_rotation_matrix(q):
    """ Konwersja kwaternionu do macierzy rotacji """
    w, x, y, z = q
    return np.array([
        [1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
        [2 * x * y + 2 * z * w, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * x * w],
        [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x ** 2 - 2 * y ** 2]
    ])


def create_pose_matrix(q, position):
    """ Tworzenie macierzy pose z kwaternionu i pozycji """
    pose_matrix = np.eye(4)
    pose_matrix[:3, :3] = quaternion_to_rotation_matrix(q)
    pose_matrix[:3, 3] = position
    return pose_matrix


def imu_to_pose(accel_data, rotationVector, delta_t):
    """ Przetwarzanie danych z IMU na macierz przesunięć 4x4. """
    print(delta_t)
    position = np.array(accel_data) * 0.5 * (delta_t ** 2)  # Proste całkowanie przyspieszenia
    rotation = quaternion_to_rotation_matrix(rotationVector)
    pose_matrix = np.eye(4)
    pose_matrix[:3, 3] = position
    pose_matrix[:3, :3] = rotation
    return pose_matrix


#
FPS = 5


class FPSCounter:
    def __init__(self):
        self.frameCount = 0
        self.fps = 0
        self.startTime = time.time()

    def tick(self):
        self.frameCount += 1
        if self.frameCount % 10 == 0:
            elapsedTime = time.time() - self.startTime
            self.fps = self.frameCount / elapsedTime
            self.frameCount = 0
            self.startTime = time.time()
        return self.fps


pipeline = dai.Pipeline()
camRgb = pipeline.create(dai.node.ColorCamera)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
depth = pipeline.create(dai.node.StereoDepth)
pointcloud = pipeline.create(dai.node.PointCloud)
sync = pipeline.create(dai.node.Sync)
xOut = pipeline.create(dai.node.XLinkOut)
xOut.input.setBlocking(False)

camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
camRgb.setIspScale(1, 3)
camRgb.setFps(FPS)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoLeft.setFps(FPS)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")
monoRight.setFps(FPS)

depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth.setLeftRightCheck(True)
depth.setExtendedDisparity(False)
depth.setSubpixel(True)
depth.setDepthAlign(dai.CameraBoardSocket.CAM_A)

monoLeft.out.link(depth.left)
monoRight.out.link(depth.right)
depth.depth.link(pointcloud.inputDepth)
camRgb.isp.link(sync.inputs["rgb"])
pointcloud.outputPointCloud.link(sync.inputs["pcl"])
sync.out.link(xOut.input)
xOut.setStreamName("out")

# Ustawienie pipeline
imu = pipeline.create(dai.node.IMU)
pointcloud = pipeline.create(dai.node.PointCloud)
xlinkOut = pipeline.create(dai.node.XLinkOut)
xlinkOut.setStreamName("imu")
imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER, 500)
imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
imu.enableIMUSensor(dai.IMUSensor.ARVR_STABILIZED_ROTATION_VECTOR, 400)

imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(10)
imu.out.link(xlinkOut.input)

# Wizualizacja
vis = o3d.visualization.Visualizer()
vis.create_window()
# axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.001, origin=[0, 0, 0])
# vis.add_geometry(axis)
line_set = o3d.geometry.LineSet()
vis.add_geometry(line_set)
pcd = o3d.geometry.PointCloud()
points_of_line = []

with dai.Device(pipeline) as device:
    imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
    q = device.getOutputQueue(name="out", maxSize=4, blocking=False)
    base_time = None

    while True:
        imuData = imuQueue.get()
        imuPackets = imuData.packets

        for imuPacket in imuPackets:
            acceleroValues = imuPacket.acceleroMeter
            rotationVector = imuPacket.rotationVector
            current_time = imuPacket.acceleroMeter.getTimestampDevice()

            if base_time is None:
                base_time = current_time

            delta_t = (current_time - base_time).total_seconds()
            base_time = current_time

            pose = imu_to_pose([acceleroValues.x, acceleroValues.y, acceleroValues.z],
                               [rotationVector.i, rotationVector.j, rotationVector.k, rotationVector.real], delta_t)
            print(pose)
            points_of_line.append(pose[:3, 3])
            # axis.transform(pose)
            # vis.add_geometry(axis)
        inMessage = q.get()
        inPointCloud = inMessage["pcl"]
        points = inPointCloud.getPoints().astype(np.float64)
        pcd.points = o3d.utility.Vector3dVector(points)
        # TODO: colors: incress read when incress y axis (hight)
        # colors = (points[:, 1].reshape(-1, 1) / 255.0).astype(np.float64)
        # pcd.colors = o3d.utility.Vector3dVector(colors)
        # pcd.transform(pose)
        # vis.add_geometry(pcd)
        # vis.poll_events()
        # vis.update_renderer()

        line_set.points = o3d.utility.Vector3dVector(points_of_line)
        if len(points_of_line) > 1:
            lines = [[j, j + 1] for j in range(len(points_of_line) - 1)]
            line_set.lines = o3d.utility.Vector2iVector(lines)
            colors = [[1, 0, 0] for _ in range(len(lines))]
            line_set.colors = o3d.utility.Vector3dVector(colors)
        vis.update_geometry(line_set)

        # vis.update_geometry(axis)
        vis.poll_events()
        vis.update_renderer()

        if cv2.waitKey(1) == ord('q'):
            break

    vis.destroy_window()
