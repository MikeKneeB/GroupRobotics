import time
from naoqi import ALProxy

#Get robot motion proxy
robotIP="127.0.0.1"
PORT=9559

motionProxy = ALProxy("ALMotion", robotIP, PORT)
    
motionProxy.setStiffnesses("Head", 1.0)
    
# Example showing how to set angles, using a fraction of max speed
names  = ["HeadYaw", "HeadPitch"]
angles  = [0.2, -0.2]
fractionMaxSpeed  = 0.2
motionProxy.setAngles(names, angles, fractionMaxSpeed)
time.sleep(3.0)

