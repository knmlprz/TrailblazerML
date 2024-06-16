import depthai as dai
from .config_oak import ConfigOak


class CameraHendler:
    """ CameraHendler class for handling the camera configuration."""

    def __init__(self, config: ConfigOak) -> None:
        """ Initialize the CameraHendler class.
        Args:
            config (dict): The configuration dictionary.
        """
        self.pipeline = dai.Pipeline()
        self.config = config
        self.setup_cameras()
        self.setup_depth()
        self.setup_pointcloud()
        self.setup_imu()

    def setup_cameras(self) -> None:
        """ Setup the cameras."""
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

    def setup_depth(self) -> None:
        """ Setup the depth cameras configuration."""
        depth_config = self.config['depth']
        self.depth = self.pipeline.create(dai.node.StereoDepth)
        print(depth_config['median_filter'])
        print(type(depth_config['median_filter']))
        self.depth.setDefaultProfilePreset(getattr(dai.node.StereoDepth.PresetMode, depth_config['preset']))
        self.depth.initialConfig.setMedianFilter(getattr(dai.MedianFilter, depth_config['median_filter']))
        self.depth.setLeftRightCheck(depth_config['left_right_check'])
        self.depth.setExtendedDisparity(depth_config['extended_disparity'])
        self.depth.setSubpixel(depth_config['subpixel'])
        self.depth.setDepthAlign(getattr(dai.CameraBoardSocket, depth_config['align_to']))

        # Konfiguracja post-processingu
        initialConfig = self.depth.initialConfig.get()

        # Konfiguracja filtra progowego na zakres 0 do 10000 mm (10 m)
        initialConfig.postProcessing.thresholdFilter.minRange = 0
        initialConfig.postProcessing.thresholdFilter.maxRange = 10000

        # Konfiguracja filtra przestrzennego
        initialConfig.postProcessing.spatialFilter.enable = True
        initialConfig.postProcessing.spatialFilter.holeFillingRadius = 2
        initialConfig.postProcessing.spatialFilter.alpha = 0.5
        initialConfig.postProcessing.spatialFilter.delta = 20

        # Konfiguracja filtra speckle
        initialConfig.postProcessing.speckleFilter.enable = True
        initialConfig.postProcessing.speckleFilter.speckleRange = 50

        # Konfiguracja filtra czasowego
        # initialConfig.postProcessing.temporalFilter.enable = True
        # # Assuming there's a method to set PersistencyMode
        # initialConfig.postProcessing.temporalFilter.PersistencyMode = dai.RawStereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_8_OUT_OF_8
        # initialConfig.postProcessing.temporalFilter.alpha = 0.4

        # Konfiguracja filtra bilateralnego
        # initialConfig.postProcessing.bilateralFilter.enable = True
        # initialConfig.postProcessing.bilateralFilter.sigma = 0.75

        # # Konfiguracja filtra decymacyjnego
        # initialConfig.postProcessing.decimationFilter.enable = True
        # initialConfig.postProcessing.decimationFilter.decimationFactor = 2

        self.depth.initialConfig.set(initialConfig)

        # Połączenie wyjść mono kamer z wejściami głębi
        self.monoLeft.out.link(self.depth.left)
        self.monoRight.out.link(self.depth.right)

        # Dodanie wyjścia dla strumienia głębi
        xout_depth = self.pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        self.depth.depth.link(xout_depth.input)

    def setup_pointcloud(self) -> None:
        """ Setup the pointcloud configuration."""
        self.pointcloud = self.pipeline.create(dai.node.PointCloud)
        self.depth.depth.link(self.pointcloud.inputDepth)
        self.sync = self.pipeline.create(dai.node.Sync)
        self.camRgb.isp.link(self.sync.inputs["rgb"])
        self.pointcloud.outputPointCloud.link(self.sync.inputs["pcl"])
        self.xOut = self.pipeline.create(dai.node.XLinkOut)
        self.sync.out.link(self.xOut.input)
        self.xOut.setStreamName("out")

    def setup_imu(self) -> None:
        """ Setup the IMU configuration."""
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
