import socket
import cPickle

TCP_IP = '127.0.0.1'
TCP_PORT = 5005
BUFFER_SIZE = 20

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)

conn, addr = s.accept()
print 'Connection address:', addr
while True:
    rec = conn.recv(BUFFER_SIZE)
    if rec:
        data = cPickle.loads(rec)
        if data is 1:
            reply = cPickle.dumps()
            conn.send(reply)

conn.close()
