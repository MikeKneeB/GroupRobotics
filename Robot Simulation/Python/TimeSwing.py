import math
import time
import SwingAPI
from naoqi import ALProxy

def smoothPeriodic(motionProxy,period,numSwings):
    #Setup timing
    numSteps=1000
    restInt=float(period)/float(numSteps)
    high = int(numSteps*period)
    #Limb and angle setup
    limbs=["Head","RLeg","LLeg"]
    angleNames=["HeadPitch","RHipPitch","LHipPitch","RKneePitch","LKneePitch"]
    for i in range(0,numSwings):
        for j in range(0,high):
            x=j*(2*math.pi/numSteps)
            headAngle=0.59*math.sin(x/period)-0.0785
            hipAngle=0.236*math.sin(x/period)+0.749
            kneeAngle=0.82*math.sin(x/period)+0.73
            angles=[headAngle,-hipAngle,-hipAngle,kneeAngle,kneeAngle]
            SwingAPI.moveLimbs(motionProxy,limbs,angleNames,angles,0.75,restInt)

def stepPeriodic(motionProxy,period,numSwings):
    restInt=float(period)/2.0
    for x in range(0,numSwings):
        SwingAPI.position1Mono(motionProxy, 0.75)
        time.sleep(restInt)
        SwingAPI.position2Mono(motionProxy, 0.75)
        time.sleep(restInt)

def Main():
    #IP, port and motion proxy
    robotIP="127.0.0.1"
    PORT=9559
    motionProxy=ALProxy("ALMotion",robotIP,PORT)
    #smoothPeriodic(motionProxy,2.435,10)
    smoothPeriodic(motionProxy,3.27,5)
    #stepPeriodic(motionProxy,3.27,5)

Main()
