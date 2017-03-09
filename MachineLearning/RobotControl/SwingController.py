import socket
import struct
from controller import Robot
from controller import PositionSensor

BUFFER_SIZE = 20

class SwingController (Robot):
  timeStep=64
  
  #Function to set up the controller
  def initialize(self):
    #Setup socket
    TCP_IP = "127.0.0.1"
    TCP_PORT = 5005
    self.s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.s.bind((TCP_IP, TCP_PORT))
    self.s.listen(1)
    print "Listening at ",TCP_IP,":",TCP_PORT
    #Setup position sensor
    self.posSen=self.getPositionSensor("swingAngle")
    self.posSen.enable(self.timeStep)
  
  #Function to accept a client
  def accept(self):
    print "Waiting for a connection"
    self.conn, self.addr = self.s.accept()
    print "Connection address:", self.addr
  
  
  #Run
  def run(self):
    self.accept()
    #MAIN LOOP
    while True:
      if self.step(self.timeStep) == -1:
        break
      #receive from client
      rec = self.conn.recv(BUFFER_SIZE)
      if rec:
        unpacker = struct.Struct('I')
        data = unpacker.unpack(rec)
        if data[0] is 1:
          #Pack reply
          values = (self.posSen.getValue())
          packer = struct.Struct('f')
          packed = packer.pack(values)
          #Send data
          self.conn.send(packed)
    #Close connection when simulation ends      
    print "Close Connection"
    self.conn.close()
      
      


#Start controller      
controller = SwingController()
controller.initialize()
controller.run()
