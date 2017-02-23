import time
import math
import SwingAPI
from naoqi import ALProxy

def Main():
    #IP, port and motion proxy
    robotIP="127.0.0.1"
    PORT=9559
    motionProxy=ALProxy("ALMotion",robotIP,PORT)
    SwingAPI.position1(motionProxy)

Main()
