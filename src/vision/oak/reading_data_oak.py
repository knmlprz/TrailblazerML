import depthai as dai
import numpy as np
import cv2
import time
import sys
import depthai as dai
import depthai as dai
import open3d as o3d
import numpy as np


def quaternion_to_rotation_matrix(q):
    """ Konwersja kwaternionu do macierzy rotacji """
    w, x, y, z = q
    return np.array([
        [1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
        [2 * x * y + 2 * z * w, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * x * w],
        [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x ** 2 - 2 * y ** 2]
    ])git

def imu_to_pose(accel_data, rotationVector, delta_t):
    """ Przetwarzanie danych z IMU na macierz przesunięć 4x4. """
    position = np.array(accel_data) * 0.5 * (delta_t ** 2)  # Proste całkowanie przyspieszenia
    rotation = quaternion_to_rotation_matrix(rotationVector)
    pose_matrix = np.eye(4)
    pose_matrix[:3, 3] = position
    pose_matrix[:3, :3] = rotation
    return pose_matrix


class CameraHendler:
    def __init__(self, config):
        self.pipeline = dai.Pipeline()
        self.config = config
        self.setup_cameras()
        self.setup_depth()
        self.setup_pointcloud()
        self.setup_imu()

    def setup_cameras(self):
        # Setup RGB Camera
        rgb_config = self.config['rgb_camera']
        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        self.camRgb.setResolution(getattr(dai.ColorCameraProperties.SensorResolution, rgb_config['resolution']))
        self.camRgb.setBoardSocket(getattr(dai.CameraBoardSocket, rgb_config['socket']))
        self.camRgb.setIspScale(*rgb_config['isp_scale'])
        self.camRgb.setFps(rgb_config['fps'])

        # Setup Mono Cameras
        mono_config = self.config['mono_cameras']
        self.monoLeft = self.pipeline.create(dai.node.MonoCamera)
        self.monoRight = self.pipeline.create(dai.node.MonoCamera)
        self.monoLeft.setResolution(getattr(dai.MonoCameraProperties.SensorResolution, mono_config['resolution']))
        self.monoRight.setResolution(getattr(dai.MonoCameraProperties.SensorResolution, mono_config['resolution']))
        self.monoLeft.setCamera("left")
        self.monoRight.setCamera("right")
        self.monoLeft.setFps(mono_config['fps'])
        self.monoRight.setFps(mono_config['fps'])

    def setup_depth(self):
        depth_config = self.config['depth']
        self.depth = self.pipeline.create(dai.node.StereoDepth)
        self.depth.setDefaultProfilePreset(getattr(dai.node.StereoDepth.PresetMode, depth_config['preset']))
        self.depth.initialConfig.setMedianFilter(getattr(dai.MedianFilter, depth_config['median_filter']))
        self.depth.setLeftRightCheck(depth_config['left_right_check'])
        self.depth.setExtendedDisparity(depth_config['extended_disparity'])
        self.depth.setSubpixel(depth_config['subpixel'])
        self.depth.setDepthAlign(getattr(dai.CameraBoardSocket, depth_config['align_to']))
        self.monoLeft.out.link(self.depth.left)
        self.monoRight.out.link(self.depth.right)

    def setup_pointcloud(self):
        self.pointcloud = self.pipeline.create(dai.node.PointCloud)
        self.depth.depth.link(self.pointcloud.inputDepth)
        self.sync = self.pipeline.create(dai.node.Sync)
        self.camRgb.isp.link(self.sync.inputs["rgb"])
        self.pointcloud.outputPointCloud.link(self.sync.inputs["pcl"])
        self.xOut = self.pipeline.create(dai.node.XLinkOut)
        self.sync.out.link(self.xOut.input)
        self.xOut.setStreamName("out")

    def setup_imu(self):
        imu_config = self.config['imu']
        self.imu = self.pipeline.create(dai.node.IMU)
        self.xlinkOut = self.pipeline.create(dai.node.XLinkOut)
        self.xlinkOut.setStreamName("imu")

        self.imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, imu_config['ACCELEROMETER_RAW'])
        self.imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, imu_config['GYROSCOPE_RAW'])
        self.imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, imu_config['ROTATION_VECTOR'])

        self.imu.setBatchReportThreshold(imu_config['batch_threshold'])
        self.imu.setMaxBatchReports(imu_config['max_batch_reports'])
        self.imu.out.link(self.xlinkOut.input)


class CameraOAK:
    def __init__(self, config):
        self.handler = CameraHendler(config)
        self.device = dai.Device(self.handler.pipeline)

    def get_data(self):
        imuQueue = self.device.getOutputQueue(name="imu", maxSize=10, blocking=False)
        pcQueue = self.device.getOutputQueue(name="out", maxSize=4, blocking=False)
        pose = None
        base_time = None
        pcd = o3d.geometry.PointCloud()

        imuData = imuQueue.get()
        imuPackets = imuData.packets
        # Get IMU data
        for imuPacket in imuPackets:
            acceleroValues = imuPacket.acceleroMeter
            rotationVector = imuPacket.rotationVector
            current_time = imuPacket.acceleroMeter.getTimestampDevice()


            if base_time is None:
                base_time = current_time

            delta_t = (current_time - base_time).total_seconds()
            base_time = current_time
            print("accc" , acceleroValues.x, acceleroValues.y, acceleroValues.z)
            pose = imu_to_pose([acceleroValues.x, acceleroValues.y, acceleroValues.z],
                               [rotationVector.i, rotationVector.j, rotationVector.k, rotationVector.real], delta_t)
        inMessage = pcQueue.get()
        inPointCloud = inMessage["pcl"]
        points = inPointCloud.getPoints().astype(np.float64)
        pcd.points = o3d.utility.Vector3dVector(points)

        return pcd, pose


# Example usage


# camera = CameraOAK(config)
# pcd, pose = camera.get_data()
# print(f"Point cloud: {pcd}")
# print(f"Pose: {pose}")

# class CameraOAK:
#
# def __init__(self, fps=5, config: Config_camera = None):
#     self.config = config
# def get_data(self):
#     while self.config.running:
#         inRgb = self.config.qRgb.get()
#         inLeft = self.config.qLeft.get()
#         inRight = self.config.qRight.get()
#         inDepth = self.config.qDepth.get()
#         inPointCloud = self.config.qPointCloud.get()
#         inMeta = self.config.qMeta.get()
#         if inRgb is not None:
#             cv2.imshow("rgb", inRgb.getCvFrame())
#         if inLeft is not None:
#             cv2.imshow("left", inLeft.getCvFrame())
#         if inRight is not None:
#             cv2.imshow("right", inRight.getCvFrame())
#         if inDepth is not None:
#             cv2.imshow("depth", (inDepth.getFrame() / 255).astype(np.uint8))
#         if inPointCloud is not None:
#             print(inPointCloud)
#         if inMeta is not None:
#             print(inMeta)
#         if cv2.waitKey(1) == ord('q'):
#             break
#         self.config.fps_counter.tick()
#         print(f"FPS: {self.config.fps_counter.fps}")
#     cv2.destroyAllWindows()
#     self.config.device.close()
#     self.config.running = False
#     print("Camera stopped")


if __name__ == "__main__":
    config = {
        'rgb_camera': {
            'resolution': 'THE_1080_P',
            'socket': 'CAM_A',
            'isp_scale': (1, 3),
            'fps': 5
        },
        'mono_cameras': {
            'resolution': 'THE_400_P',
            'fps': 5
        },
        'depth': {
            'preset': 'HIGH_DENSITY',
            'median_filter': 'KERNEL_7x7',
            'left_right_check': True,
            'extended_disparity': False,
            'subpixel': True,
            'align_to': 'CAM_A'
        },
        'imu': {
            'ACCELEROMETER_RAW': 500,
            'GYROSCOPE_RAW': 400,
            'ROTATION_VECTOR': 400,
            'batch_threshold': 1,
            'max_batch_reports': 10
        }
    }
    camera = CameraOAK(config)
    while True:
        pcd, pose = camera.get_data()
        print(f"Point cloud: {pcd}")
        print(f"Pose: {pose}")
