import depthai as dai


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

        self.imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER, imu_config['ACCELEROMETER_RAW'])
        self.imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, imu_config['GYROSCOPE_RAW'])
        self.imu.enableIMUSensor(dai.IMUSensor.ARVR_STABILIZED_ROTATION_VECTOR, imu_config['ROTATION_VECTOR'])

        self.imu.setBatchReportThreshold(imu_config['batch_threshold'])
        self.imu.setMaxBatchReports(imu_config['max_batch_reports'])
        self.imu.out.link(self.xlinkOut.input)
