import socket
import struct
import numpy as np

class DashboardSocket:

    def __init__(self, client, host: str, port: int):
        """
        host: address of server
        port: port on server to connect to
        """
        self._client = client
        self._socket = socket.socket()
        self._host = host
        self._port = port
        
        self._conn = None
        if not self._client:
            self._socket.bind((self._host, self._port))
            self._socket.listen()

    def connect(self):
        if self._client:
            print(f"Client connecting to {self._host} and port {self._port} ...")
            self._socket.connect((self._host, self._port))
            print(f"Client connected")
        else:
            print(f"Server connecting to {self._host} and port {self._port} ...")
            conn, addr = self._socket.accept()
            self._conn = conn
            print(f"Server connected")


    def close(self):
        """Close socket."""
        self._socket.close()
        print("Connection closed")


    def send(self, s):
        """Write to server."""
        if self._client:
            self._socket.sendall(struct.pack("i", len(s)))
            self._socket.sendall(bytes(s, 'utf-8'))


    def receive(self):
        """Read from server."""
        if not self._client:
            l_msg = self._conn.recv(4)
            if l_msg:
                length = struct.unpack("i", l_msg)[0]
                data = self._conn.recv(length).decode('utf-8').split()
                return data
            print("Client Disconnected")
            return None

def particles_to_str():
    c=''
    for _ in range(10):
        a = np.random.uniform(0, 100)
        b = np.random.uniform(0, 100)
        d = np.random.uniform(0, 1)
        c += f"{a:05.3f} {b:05.3f} {d:05.3f} \n\r"
    return c

def str_to_particles(data, n_landmarks):
    x, y, w, landmarks = [], [], [], []
    particle_length = n_landmarks * 2 + 3 
    print("DATA LENGTH", len(data))
    if len(data) < 3+n_landmarks*2:
        return [0], [0], [0], [[0, 0]]

    for i in range(0, len(data), particle_length):
        print("INDEX", i)
        a, b, c = float(data[i]), float(data[i+1]), float(data[i+2])
        x.append(a)
        y.append(b)
        w.append(c)
        current = []
        for j in range(n_landmarks):
            current.append([float(data[i+3+2*j]), float(data[i+4+2*j])])
        landmarks.append(current)
    return np.array(x), np.array(y), np.array(w), np.array(landmarks)
