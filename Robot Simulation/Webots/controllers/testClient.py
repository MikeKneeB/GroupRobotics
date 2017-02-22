import socket
import cPickle

class SwingSensor:
    def __init__(self, TCP_IP, TCP_PORT):
        self.BUFFER_SIZE = 20
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((TCP_IP, TCP_PORT))

    def getUpdate(self):
        self.s.send(cPickle.dumps(1))
        rec = self.s.recv(self.BUFFER_SIZE)
        return cPickle.loads(rec)

    def closeConnection(self):
        self.s.close()


swingProxy = SwingSensor('127.0.0.1', 5005)
data = swingProxy.getUpdate()
print data
data = swingProxy.getUpdate()
print data
data = swingProxy.getUpdate()
print data
data = swingProxy.getUpdate()
print data
data = swingProxy.getUpdate()
print data
