from abc import ABC, abstractmethod
import threading
import time
from enum import Enum
import struct

import serial
import socket

from .utils import DataType


PACKET_HEADER_FORMAT = "If"   # type + timestamp
IMU_DATA_FORMAT = "I f f f f f f f"  # timestamp, temp, gx, gy, gz, ax, ay, az
IMU_PACKET_FORMAT = PACKET_HEADER_FORMAT + IMU_DATA_FORMAT
IMU_PACKET_SIZE = struct.calcsize(IMU_PACKET_FORMAT)
UWB_DATA_FORMAT = "b B f f f"  # anchor_id, nlos, distance, azimuth, elevation
UWB_PACKET_FORMAT = PACKET_HEADER_FORMAT + UWB_DATA_FORMAT
UWB_PACKET_SIZE = struct.calcsize(UWB_PACKET_FORMAT)
POSE_DATA_FORMAT = "f f f f f f f"  # x, y, z, qx, qy, qz, qw
POSE_PACKET_FORMAT = PACKET_HEADER_FORMAT + POSE_DATA_FORMAT
POSE_PACKET_SIZE = struct.calcsize(POSE_PACKET_FORMAT)


class Receiver(ABC):
    def __init__(self, viewer=None):
        self.viewer = viewer
        self.running = False
        self.thread = None

    @abstractmethod
    def open(self):
        pass

    @abstractmethod
    def close(self):
        pass

    @abstractmethod
    def receive(self):
        pass
    
    @abstractmethod
    def _parse_data(self, data):
        pass

    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()

    def _run(self):
        while self.running:
            data = self.receive()
            if data and self.viewer: # need to refactor
                data_type, data_dict = self._parse_data(data)
                self.viewer.draw_data(data_type, data_dict)


class SerialReceiver(Receiver):
    def __init__(self, port: str, baudrate: int, viewer=None):
        super().__init__(viewer)
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None

    def open(self):
        self.serial_port = serial.Serial(self.port, self.baudrate, timeout=10)
        print(f"Listening on {self.port} at {self.baudrate} baud...")

    def close(self):
        self.stop()
        if self.serial_port:
            self.serial_port.close()
            print("Serial port closed.")

    def receive(self):
        if self.serial_port:
            return self.serial_port.readline().decode('utf-8', errors="ignore").strip()
        return None
    
    def _parse_data(self, data):
        data_type_str = data[1:5].replace(" ", "")
        data_type = self._parse_data_type(data_type_str)
        if data_type == DataType.UNKNOWN:
            print(f"Unknown data type: {data_type_str}")
            return None

        data = data[6:].replace(" ", "")
        data_dict = {key: value for key, value in (item.split("=") for item in data.split(","))}
        return data_type, data_dict

    def _parse_data_type(self, data_type_str):
        if data_type_str == "imu":
            return DataType.IMU
        elif data_type_str == "uwb":
            return DataType.UWB
        elif data_type_str == "pose":
            return DataType.POSE
        else:
            return DataType.UNKNOWN


class UdpReceiver(Receiver):
    def __init__(self, ip: str, port: int, viewer=None):
        super().__init__(viewer)
        self.ip = ip
        self.port = port
        self.sock = None

    def open(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.ip, self.port))
        print(f"UDP socket opened on {self.ip}:{self.port}")

    def close(self):
        self.stop()
        if self.sock:
            self.sock.close()
            print("UDP socket closed.")

    def receive(self):
        if self.sock:
            data, _ = self.sock.recvfrom(128)
            return data
        return None
    
    def _parse_data(self, data):
        if len(data) < struct.calcsize(PACKET_HEADER_FORMAT):
            print(f"Invalid packet size : {len(data)}")
            return None
        data_type_int, timestamp = struct.unpack(PACKET_HEADER_FORMAT, data[:8])
        data_type = DataType(data_type_int)
        if data_type == DataType.IMU:
            imu_values = struct.unpack(IMU_PACKET_FORMAT, data[:IMU_PACKET_SIZE])
            data_dict = {
                "timestamp"   : imu_values[1],
                "temperature" : imu_values[3],
                "gx"          : imu_values[4],
                "gy"          : imu_values[5],
                "gz"          : imu_values[6],
                "ax"          : imu_values[7],
                "ay"          : imu_values[8],
                "az"          : imu_values[9]
            }
        elif data_type == DataType.UWB:
            uwb_values = struct.unpack(UWB_PACKET_FORMAT, data[:UWB_PACKET_SIZE])
            data_dict = {
                "timestamp": uwb_values[1],
                "anchor_id": uwb_values[2],
                "nlos"     : uwb_values[3],
                "distance" : uwb_values[4],
                "azimuth"  : uwb_values[5],
                "elevation": uwb_values[6]
            }
        elif data_type == DataType.POSE:
            pose_values = struct.unpack(POSE_PACKET_FORMAT, data[:POSE_PACKET_SIZE])
            data_dict = {
                "timestamp": pose_values[1],
                "x"        : pose_values[2],
                "y"        : pose_values[3],
                "z"        : pose_values[4],
                "qx"       : pose_values[5],
                "qy"       : pose_values[6],
                "qz"       : pose_values[7],
                "qw"       : pose_values[8]
            }
        return data_type, data_dict
