import depthai as dai
import open3d as o3d
import numpy as np
from imu_tracker import ImuTracker
from camera_hendler import CameraHendler


class CameraOAK:
    def __init__(self, config):
        self.handler = CameraHendler(config)
        self.device = dai.Device(self.handler.pipeline)
        self.base_time = None
        self.imu_tracker = ImuTracker()

    def get_data(self):
        imuQueue = self.device.getOutputQueue(name="imu", maxSize=50, blocking=False)
        pcQueue = self.device.getOutputQueue(name="out", maxSize=4, blocking=False)
        pose = None
        pcd = o3d.geometry.PointCloud()

        imuData = imuQueue.get()
        imuPackets = imuData.packets
        # Get IMU data
        for imuPacket in imuPackets:
            acceleroValues = imuPacket.acceleroMeter
            rotationVector = imuPacket.rotationVector
            current_time = imuPacket.acceleroMeter.getTimestampDevice()

            if self.base_time is None:
                self.base_time = current_time

            delta_t = (current_time - self.base_time).total_seconds()
            self.base_time = current_time
            pose = self.imu_tracker.update([acceleroValues.x, acceleroValues.y, acceleroValues.z],
                                           [rotationVector.i, rotationVector.j, rotationVector.k, rotationVector.real],
                                           delta_t)
        inMessage = pcQueue.get()
        inPointCloud = inMessage["pcl"]
        points = inPointCloud.getPoints().astype(np.float64)
        pcd.points = o3d.utility.Vector3dVector(points)

        # inColor = inMessage["rgb"]
        # cvColorFrame = inColor.getCvFrame()
        # # cvRGBFrame = cv2.cvtColor(cvColorFrame, cv2.COLOR_BGR2RGB)
        # cv2.imshow("color", cvColorFrame)

        return pcd, pose
