import time
from naoqi import ALProxy


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

def moveHead(motionProxy,angle,restInt):
    limbs=["Head"]
    angleNames=["HeadPitch"]
    angles=[angle]
    speed=0.5
    moveLimbs(motionProxy,limbs,angleNames,angles,speed,restInt)
