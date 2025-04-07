import argparse

import toml
import numpy as np

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