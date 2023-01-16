# Client
from dashboard_socket import DashboardSocket, particles_to_str
import time

HOST = '127.0.0.1'# localhost
PORT = 65432 # random number

socket = DashboardSocket(True, HOST, PORT)
socket.connect()
try:
    for i in range(10):
        s = particles_to_str()
        socket.send(s)
        time.sleep(1)
finally:
    socket.close()

"""
import socket
import numpy as np
import time
import struct

HOST = '127.0.0.1'# localhost
PORT = 65432 # random number

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    print("CONNECTED")
    while True:
        c=''
        for i in range(10):
            a = np.random.uniform(0, 100)
            b = np.random.uniform(0, 100)
            c += f"{a:05.3f} {b:05.3f}\n\r"
        input("Press to continue")
        s.sendall(struct.pack("i", len(c)))
        s.sendall(bytes(c, 'utf-8'))
        print("SENDING", c, len(c))
        if s == "END":
            break
        time.sleep(0.5)


"""
