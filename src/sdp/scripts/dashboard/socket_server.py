from dashboard_socket import DashboardSocket, str_to_particles

HOST = '127.0.0.1'
PORT = 65432

socket = DashboardSocket(False, HOST, PORT)
socket.connect()
try:
    while True:
        data = socket.receive()
        if not data:
            break
        data = str_to_particles(data)
        print(data)
finally:
    socket.close()

"""
import socket
import struct

HOST = '127.0.0.1'
PORT = 65432

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    while True:
        try:
            conn, addr = s.accept()
            print("CONNECTED")
            with conn:
                while True:
                    input("Press to receive data")
                    length = struct.unpack("i", conn.recv(4))[0]
                    print(length)
                    data = conn.recv(length).decode('utf-8').split()
                    for i in range(0, len(data), 2):
                        a, b = float(data[i]), float(data[i+1])
                        print(a, b)
                    
                    if not data:
                        break
                    print("RECEIVED", data)
        except KeyboardInterrupt:
            print("SHUTTING DOWN")
            break
        except Exception as e:
            print(e)
            input()
"""
