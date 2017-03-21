import math
import time
import SwingAPI
from naoqi import ALProxy

def smoothPeriodic(motionProxy,period,numSwings):
    i = 0
    #Setup timing
    numSteps=100
    restInt=float(period)/float(numSteps)
    #Limb and angle setup
    limbs=["Head","RLeg","LLeg"]
    angleNames=["HeadPitch","RHipPitch","LHipPitch","RKneePitch","LKneePitch"]
    while i < numSwings:
        print i
        t = 0
        while t < period:
            x = 2*math.pi*t/period
            headAngle = 0.59 * math.sin(x) - 0.0785
            hipAngle = 0.236 * math.sin(x) + 0.749
            kneeAngle = 0.82 * math.sin(x) + 0.73
            angles = [headAngle, -hipAngle, -hipAngle, kneeAngle, kneeAngle]
            SwingAPI.moveLimbs(motionProxy, limbs, angleNames, angles, 0.75, restInt)
            t += restInt
        i += 1
def stepPeriodic(motionProxy,period,numSwings):
    restInt=float(period)/2.0
    for x in range(0,numSwings):
        SwingAPI.position1Dynamic(motionProxy, 0.75)
        time.sleep(restInt)
        SwingAPI.position2Dynamic(motionProxy, 0.75)
        time.sleep(restInt)

def Main():
    #IP, port and motion proxy
    robotIP="127.0.0.1"
    PORT=9559
    motionProxy=ALProxy("ALMotion",robotIP,PORT)
    smoothPeriodic(motionProxy,2.5,120)

Main()
