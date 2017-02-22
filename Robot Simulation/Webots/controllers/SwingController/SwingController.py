import socket
import cPickle
from controller import Robot

BUFFER_SIZE = 20

class SwingController (Robot):
  def __init__(self):
    TCP_IP = "127.0.0.1"
    TCP_PORT = 5005
    
    
    self.s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.s.bind((TCP_IP, TCP_PORT))
    self.s.listen(1)
    
    print "Listening at ",TCP_IP,":",TCP_PORT
  
  def accept(self):
    print "Waiting for a connection"
    self.conn, self.addr = self.s.accept()
    print "Connection address:", self.addr
  
  def run(self):
    self.accept()
    while True:
      rec = self.conn.recv(BUFFER_SIZE)
      if rec:
        data = cPickle.loads(rec)
        if data is 1:
          reply = cPickle.dumps(12)
          self.conn.send(reply)
    self.conn.close()
      
      
      
controller = SwingController()
controller.run()
