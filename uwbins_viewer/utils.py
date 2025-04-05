from dataclasses import dataclass
from enum import Enum

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

class DataType(Enum):
    IMU = 0
    UWB = 1
    POSE = 2
    UNKNOWN = 255