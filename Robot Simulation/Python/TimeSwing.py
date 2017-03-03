import math
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
            SwingAPI.moveLimbs(motionProxy,limbs,angleNames,angles,0.5,restInt)

def stepPeriodic(motionProxy,period,numSwings):
    restInt=float(period)/2.0
    for x in range(0,numSwings):
        SwingAPI.moveLegs(motionProxy,LEGS_MAX,restInt)
        SwingAPI.moveLegs(motionProxy,LEGS_MIN,restInt)

def Main():
    #IP, port and motion proxy
    robotIP="127.0.0.1"
    PORT=9559
    motionProxy=ALProxy("ALMotion",robotIP,PORT)
    #smoothPeriodic(motionProxy,1/2.435,10)
    smoothPeriodic(motionProxy,1.5,5)

Main()
