from typing import TypedDict, List
import json


class RGB_Camera(TypedDict):
    resolution: str
    socket: str
    isp_scale: List[int]
    fps: int


class Mono_Cameras(TypedDict):
    resolution: str
    fps: int


class Depth(TypedDict):
    preset: str
    median_filter: str
    left_right_check: bool
    extended_disparity: bool
    subpixel: bool
    align_to: str


class IMU(TypedDict):
    ACCELEROMETER_RAW: int
    GYROSCOPE_RAW: int
    ROTATION_VECTOR: int
    batch_threshold: int
    max_batch_reports: int


class ConfigOak(TypedDict):
    rgb_camera: RGB_Camera
    mono_cameras: Mono_Cameras
    depth: Depth
    imu: IMU


def load_config(path: str) -> ConfigOak:
    with open(path, "r") as f:
        data = json.load(f)
        config: ConfigOak = data
        return config
