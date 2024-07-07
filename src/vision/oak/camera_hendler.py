import depthai as dai
from .config_oak import ConfigOak


class CameraHendler:
    """CameraHendler class for handling the camera configuration."""

    def __init__(self, config: ConfigOak) -> None:
        """Initialize the CameraHendler class.
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
        """Setup the cameras."""
        # Setup RGB Camera
        rgb_config = self.config["rgb_camera"]
        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        self.camRgb.setResolution(
            getattr(
                dai.ColorCameraProperties.SensorResolution, rgb_config["resolution"]
            )
        )
        self.camRgb.setBoardSocket(getattr(dai.CameraBoardSocket, rgb_config["socket"]))
        self.camRgb.setIspScale(*rgb_config["isp_scale"])
        self.camRgb.setFps(rgb_config["fps"])

        # Setup Mono Cameras
        mono_config = self.config["mono_cameras"]
        self.monoLeft = self.pipeline.create(dai.node.MonoCamera)
        self.monoRight = self.pipeline.create(dai.node.MonoCamera)
        self.monoLeft.setResolution(
            getattr(
                dai.MonoCameraProperties.SensorResolution, mono_config["resolution"]
            )
        )
        self.monoRight.setResolution(
            getattr(
                dai.MonoCameraProperties.SensorResolution, mono_config["resolution"]
            )
        )
        self.monoLeft.setCamera("left")
        self.monoRight.setCamera("right")
        self.monoLeft.setFps(mono_config["fps"])
        self.monoRight.setFps(mono_config["fps"])

    def setup_depth(self):
        """Setup the depth cameras configuration."""
        depth_config = self.config["depth"]
        self.depth = self.pipeline.create(dai.node.StereoDepth)

        # Apply depth settings
        self.depth.setDefaultProfilePreset(
            getattr(dai.node.StereoDepth.PresetMode, depth_config["preset"])
        )
        self.depth.initialConfig.setMedianFilter(
            getattr(dai.MedianFilter, depth_config["median_filter"])
        )
        self.depth.setLeftRightCheck(depth_config["left_right_check"])
        # self.depth.setExtendedDisparity(depth_config["extended_disparity"])
        # self.depth.setSubpixel(depth_config["subpixel"])
        self.depth.setDepthAlign(
            getattr(dai.CameraBoardSocket, depth_config["align_to"])
        )

        # Get the current configuration to modify
        initialConfig = self.depth.initialConfig.get()

        # Apply post processing settings
        # Threshold filter
        threshold_filter = depth_config["threshold_filter"]
        initialConfig.postProcessing.thresholdFilter.minRange = threshold_filter[
            "min_range"
        ]
        initialConfig.postProcessing.thresholdFilter.maxRange = threshold_filter[
            "max_range"
        ]

        # Spatial filter
        spatial_filter = depth_config["spatial_filter"]
        initialConfig.postProcessing.spatialFilter.enable = spatial_filter["enable"]
        initialConfig.postProcessing.spatialFilter.holeFillingRadius = spatial_filter[
            "hole_filling_radius"
        ]
        initialConfig.postProcessing.spatialFilter.alpha = spatial_filter["alpha"]
        initialConfig.postProcessing.spatialFilter.delta = spatial_filter["delta"]

        # Speckle filter
        speckle_filter = depth_config["speckle_filter"]
        initialConfig.postProcessing.speckleFilter.enable = speckle_filter["enable"]
        initialConfig.postProcessing.speckleFilter.speckleRange = speckle_filter[
            "speckle_range"
        ]

        self.depth.initialConfig.set(initialConfig)

        self.monoLeft.out.link(self.depth.left)
        self.monoRight.out.link(self.depth.right)

        xout_depth = self.pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        self.depth.depth.link(xout_depth.input)

        # TODO temporalFilter
        # initialConfig.postProcessing.temporalFilter.enable = True
        # # Assuming there's a method to set PersistencyMode
        # initialConfig.postProcessing.temporalFilter.PersistencyMode = dai.RawStereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_8_OUT_OF_8
        # initialConfig.postProcessing.temporalFilter.alpha = 0.4

        # TODO bilateralFilter
        # initialConfig.postProcessing.bilateralFilter.enable = True
        # initialConfig.postProcessing.bilateralFilter.sigma = 0.75

        # TODO decimationFilter
        # initialConfig.postProcessing.decimationFilter.enable = True
        # initialConfig.postProcessing.decimationFilter.decimationFactor = 2

    def setup_pointcloud(self) -> None:
        """ Setup the pointcloud configuration."""
        self.pointcloud = self.pipeline.create(dai.node.PointCloud)
        self.pointcloud.initialConfig.setSparse(True)
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
        selected_data_type = imu_config['selected_data_type']
        self.imu = self.pipeline.create(dai.node.IMU)
        self.xlinkOut = self.pipeline.create(dai.node.XLinkOut)
        self.xlinkOut.setStreamName("imu")

        for sensor_name, frequency in selected_data_type.items():
            sensor_enum = getattr(dai.IMUSensor, sensor_name)  # Konwertuj nazwę sensora na odpowiedni enum
            self.imu.enableIMUSensor(sensor_enum, frequency)

        self.imu.setBatchReportThreshold(imu_config["batch_threshold"])
        self.imu.setMaxBatchReports(imu_config["max_batch_reports"])
        self.imu.out.link(self.xlinkOut.input)