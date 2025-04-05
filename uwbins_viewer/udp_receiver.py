import socket
import struct

# Cのpacket_t構造体のバイナリフォーマット
PACKET_HEADER_FORMAT = "B f"   # type (uint8) + timestamp (float)
IMU_DATA_FORMAT = "I f f f f f f f"  # timestamp, temp, gx, gy, gz, ax, ay, az
IMU_PACKET_FORMAT = PACKET_HEADER_FORMAT + IMU_DATA_FORMAT  # 合計37バイト
IMU_PACKET_SIZE = struct.calcsize(IMU_PACKET_FORMAT)
print(f"IMU Packet Size: {IMU_PACKET_SIZE} bytes")

# 受信設定
UDP_IP = "192.168.11.11"  # すべてのインターフェースで受信
UDP_PORT = 50000

# UDPソケットの作成
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening on {UDP_IP}:{UDP_PORT}...")
try:
    data, addr = sock.recvfrom(1024)  # 十分なバッファサイズで受信
    # バッファに残っているデータを消費する
    print("Clearing the buffer...")
except socket.timeout:
    # バッファに残っている古いデータをクリア
    print("Buffer cleared or no data received.")
while True:
    data, addr = sock.recvfrom(128)
    print(f"Received packet size: {len(data)} bytes from {addr}")
    if len(data) < struct.calcsize(PACKET_HEADER_FORMAT):
        print("Invalid packet size")
        continue

    packet_type, timestamp = struct.unpack(PACKET_HEADER_FORMAT, data[:8])
    if packet_type == 0:  # PACKET_TYPE_IMU
        imu_values = struct.unpack(IMU_PACKET_FORMAT, data)
        print(f"IMU Packet: Timestamp={imu_values[1]}, Temp={imu_values[2]}, "
              f"Gx={imu_values[3]}, Gy={imu_values[4]}, Gz={imu_values[5]}, "
              f"Ax={imu_values[6]}, Ay={imu_values[7]}, Az={imu_values[8]}")
    else:
        print(f"Unknown Packet Type: {packet_type}")
