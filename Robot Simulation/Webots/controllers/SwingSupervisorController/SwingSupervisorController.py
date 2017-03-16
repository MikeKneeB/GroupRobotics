import socket
import select
import time
import threading
import struct
from controller import Supervisor
from controller import PositionSensor

BUFFER_SIZE = 20

class SwingController (Supervisor):
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
    
    t = threading.Thread(target=self.stepSim)
    t.start()
  
  #Function to accept a client
  def accept(self):
    print "Waiting for a connection"
    self.conn, self.addr = self.s.accept()
    print "Connection address:", self.addr
    #Set the connnection to non blocking
    self.conn.setblocking(0)
  
  def stepSim(self):
    while True:
      if self.step(self.timeStep) == -1:
        break
      time.sleep(0.01)
  
  #Run
  def run(self):
    self.accept()
    inputs = [ self.conn ]
    outputs = [ ]
    count = 0
    while inputs:
      #Check if any information in buffer  
      time.sleep(0.01)
      readable, writable, exceptional = select.select(inputs, outputs, inputs)
      for sock in readable:
        #Read from connection with data
        data = sock.recv(BUFFER_SIZE)
        if data:
          if data == "update":
          #Pack reply
            values = (self.posSen.getValue())
            packer = struct.Struct('f')
            packed = packer.pack(values)
            #Send data
            sock.send(packed)
          elif data == "revert":
            #Revert the simulation to its intial conditions
            self.simulationRevert()
    
    #Close connection when simulation ends      
    print "Close Connection"
    self.conn.close()
      
      


#Start controller      
controller = SwingController()
controller.initialize()
controller.run()
