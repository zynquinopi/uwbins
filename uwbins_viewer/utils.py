from dataclasses import dataclass
import numpy as np

@dataclass
class Imu:
    timestamp: float
    temperature: float
    acc: np.ndarray
    gyro: np.ndarray


@dataclass
class Uwb:
    timestamp: float
    anchor_id: int
    nlos: int
    distance: float
    azimuth: float
    elevation: float


@dataclass
class Pose:
    position: np.ndarray
    rotation: np.ndarray

@dataclass
class AnchorPose:
    id: int
    position: np.ndarray
    rotation: np.ndarray