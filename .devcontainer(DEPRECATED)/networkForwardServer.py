#!/usr/bin/env python3
import socket
import threading
import time

# Camera static IP
CAMERA_IP = "192.168.1.102"
# UDP ports required by camera
UDP_PORTS = [9007, 9008, 9009]

def udp_forward(listen_port, target_ip, target_port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', listen_port))
    print(f"[{time.strftime('%H:%M:%S')}] [UDP Proxy] Listening on {listen_port} -> {target_ip}:{target_port}")

    while True:
        data, addr = sock.recvfrom(4096)
        # DEBUG: log when packet is received
        print(f"[{time.strftime('%H:%M:%S')}] [UDP Proxy] Received {len(data)} bytes from {addr} on port {listen_port}")
        sock.sendto(data, (target_ip, target_port))

# Launch forwarding threads
for port in UDP_PORTS:
    threading.Thread(target=udp_forward, args=(port, CAMERA_IP, port), daemon=True).start()

# Keep alive
threading.Event().wait()