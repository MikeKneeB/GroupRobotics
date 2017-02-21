import time
import math
from naoqi import ALProxy

LEGS_MIN=0
LEGS_MAX=1.2

def moveLimbs(motionProxy,limbs,angleNames,angles,speed,restInt):
    motionProxy.setStiffnesses(limbs,1.0)
    motionProxy.setAngles(angleNames,angles,speed)
    time.sleep(restInt)

def moveLegs(motionProxy,angle,restInt):
    limbs=["RLeg","LLeg"]
    angleNames=["RKneePitch","LKneePitch"]
    angles=[angle,angle]
    speed=0.5
    moveLimbs(motionProxy,limbs,angleNames,angles,speed,restInt)

def moveHips(motionProxy,angle,restInt):
    limbs=["RLeg","LLeg"]
    angleNames=["RHipPitch","LHipPitch"]
    angles=[angle,angle]
    speed=0.5
    moveLimbs(motionProxy,limbs,angleNames,angles,speed,restInt)

def smoothPeriodic(motionProxy,period,numSwings):
    numSteps=100
    restInt=float(period)/float(numSteps)
    high = int(numSteps*period)
    for i in range(0,numSwings):
        for j in range(0,high):
            x=j*(2*math.pi/numSteps)
            angle=(LEGS_MAX/2)+(LEGS_MAX/2)*math.sin(x/period)
            moveLegs(motionProxy,angle,restInt)

def stepPeriodic(motionProxy,period,numSwings):
    restInt=float(period)/2.0
    for x in range(0,numSwings):
        moveLegs(motionProxy,LEGS_MAX,restInt)
        moveLegs(motionProxy,LEGS_MIN,restInt)

def Main():
    #IP, port and motion proxy
    robotIP="127.0.0.1"
    PORT=9559
    motionProxy=ALProxy("ALMotion",robotIP,PORT)
    moveHips(motionProxy,-1,1)

Main()
