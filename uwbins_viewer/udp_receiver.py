
import socket

UDP_IP = "0.0.0.0"  # すべてのネットワークインターフェースで受信
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on port {UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(1024)  # 1024バイトまで受信
    print(f"Received from {addr}: {data.decode('utf-8')}")