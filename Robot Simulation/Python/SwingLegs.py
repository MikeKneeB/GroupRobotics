import SwingAPI
import time
from naoqi import ALProxy

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