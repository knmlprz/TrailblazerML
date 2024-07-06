import depthai as dai
from .config_oak import ConfigOak

class CameraHendler:
    """"CameraHendler class for handling the camera configuration."""

    def __init__(self, config: ConfigOak) -> None:
        """Initialize the CameraHendler class.
        Args:
            config (ConfigOak): The configuration dictionary.
        """
        self.pipeline = dai.Pipeline()
        self.config = config
        self.setup_cameras()
        self.setup_depth()
        self.setup_imu()

    def setup_cameras(self) -> None:
        """Setup the cameras."""
