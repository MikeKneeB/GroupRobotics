import socket
import struct

#   Author: Harry Withers
#   Date:   23/02/2017
#
#   This class acts as a proxy when connecting to the virtual swing.
#   It has 2 functions, one to get the position of the swing and one
#   to revert the Webots world to its initial conditions.

class SwingProxy:
    def __init__(self, TCP_IP, TCP_PORT):
        self.BUFFER_SIZE = 20
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((TCP_IP, TCP_PORT))

    def get_angle(self):
        #Send request
        try:
            self.s.send("update")
        except socket.error, e:
            print "Disconnected from swing"
            return 999
        #Unpack reply
        try:
            rec = self.s.recv(self.BUFFER_SIZE)
            unpacker = struct.Struct('f')
            return unpacker.unpack(rec)[0]
        except socket.error, e:
            print "Disconnected from swing"
            return 999

    def revert_world(self):
        # Send request
        try:
            self.s.send("revert")
        except socket.error, e:
            print "Could not revert world"

    def closeConnection(self):
        self.s.close()

