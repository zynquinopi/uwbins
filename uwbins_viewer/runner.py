import argparse
import toml

import numpy as np
import pandas as pd

from .viewer import UwbInsViewer
from .receiver import UdpReceiver, SerialReceiver
from .utils import *


def load_anchor_poses(toml_args) -> list[AnchorPose]:
    anchors = []
    for _, anchor in toml_args.get("uwb", {}).items():
        anchors.append(
            AnchorPose(
                id=int(anchor.get("id", -1)),
                position=np.array(anchor.get("position", [0.0, 0.0, 0.0])),
                rotation=np.array(anchor.get("rotation", [0.0, 0.0, 0.0, 1.0]))
            )
        )
    return anchors


def online(args: argparse.Namespace) -> None:
    toml_args = toml.load(args.config)


    anchor_poses = load_anchor_poses(toml_args)
    viewer = UwbInsViewer(anchor_poses)
    
    interface = toml_args["receiver"]["interface"]
    if interface == "serial":
        serial_config = toml_args["receiver"]["serial"]
        receiver = SerialReceiver(serial_config["device"], serial_config["baudrate"], viewer=viewer)
    elif interface == "socket":
        socket_config = toml_args["receiver"]["socket"]
        receiver = UdpReceiver(socket_config["ip"], socket_config["port"], viewer=viewer)
    receiver.open()
    receiver.start()
    input("Press Enter⏎ to stop viewer\n")


def offline(args: argparse.Namespace) -> None:
    toml_args = toml.load(args.config)

    anchor_poses = load_anchor_poses(toml_args)
    viewer = UwbInsViewer(anchor_poses)

    df = pd.read_csv("../data/imu.txt")
    timestamp = df['timestamp[us]'].values
    temp = df['temp[celsius]'].values
    acc = df[['ax[m/s^2]', 'ay[m/s^2]', 'az[m/s^2]']].values
    gyro = df[['gx[rad/s]', 'gy[rad/s]', 'gz[rad/s]']].values
    viewer.draw_data_batch(
        DataType.IMU,
        [Imu(timestamp[i], temp[i], acc[i], gyro[i]) for i in range(len(timestamp))]
    )

    df = pd.read_csv("../data/uwb.txt")
    timestamp = df['timestamp[us]'].values
    anchor_id = df['anchor_id'].values
    nlos = df['nlos'].values
    distance = df['distance[m]'].values
    azimuth = df['azimuth[deg]'].values
    elevation = df['elevation[deg]'].values
    viewer.draw_data_batch(
        DataType.UWB,
        [Uwb(timestamp[i], anchor_id[i], nlos[i], distance[i], azimuth[i], elevation[i]) for i in range(len(timestamp))]
    )

    df = pd.read_csv("../data/pose.txt")
    timestamp = df['timestamp[us]'].values
    position = df[['x[m]', 'y[m]', 'z[m]']].values
    quaternion = df[['qx', 'qy', 'qz', 'qw']].values
    viewer.draw_data_batch(
        DataType.POSE,
        [Pose(timestamp[i], position[i], quaternion[i]) for i in range(len(timestamp))]
    )