import socket
import struct

class SwingProxy:
    def __init__(self, TCP_IP, TCP_PORT):
        self.BUFFER_SIZE = 20
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((TCP_IP, TCP_PORT))

    def get_angle(self):
        #Pack instruction
        values=(1)
        packer = struct.Struct('I')
        packed = packer.pack(values)
        self.s.send(packed)
        #Unpack reply
        rec = self.s.recv(self.BUFFER_SIZE)
        unpacker = struct.Struct('f')
        return unpacker.unpack(rec)[0]

    def closeConnection(self):
        self.s.close()

