import SwingAPI
import time
from naoqi import ALProxy

#   Author: Harry Withers
#   Date:   21/02/2017
#
#   This is a simply file to test the robots motion between 2 positions
#   using SwingAPI.

def Main():
    #IP, port and motion proxy
    robotIP = "127.0.0.1"
    PORT = 9559
    motionProxy = ALProxy("ALMotion",robotIP,PORT)

    while True:
        SwingAPI.position2Dynamic(motionProxy,0.75)
        time.sleep(3)
        SwingAPI.position1Dynamic(motionProxy,0.75)
        time.sleep(3)

Main()