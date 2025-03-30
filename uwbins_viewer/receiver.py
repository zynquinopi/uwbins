from abc import ABC, abstractmethod
import threading
import time

import serial
import socket


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
                data_type = data[1:5].replace(" ", "")
                if (data_type not in ["imu", "uwb", "pose"]) or len(data) > 100:
                    continue
                print(data)
                data = data[6:].replace(" ", "")
                data_dict = {key: value for key, value in (item.split("=") for item in data.split(","))}
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
            data, _ = self.sock.recvfrom(1024)
            return data.decode('utf-8').strip()
        return None