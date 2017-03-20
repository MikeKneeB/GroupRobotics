import math
import time
import SwingAPI
import SwingProxy
from naoqi import ALProxy

def smoothPeriodicMax(motionProxy, swingProxy, period, numSwings):
    maxAngle = 0
    #Setup timing
    numSteps=1000
    restInt=float(period)/float(numSteps)
    #Limb and angle setup
    limbs=["Head","RLeg","LLeg"]
    angleNames=["HeadPitch","RHipPitch","LHipPitch","RKneePitch","LKneePitch"]
    for i in range(0,numSwings):
        t = 0
        while t < period:
            x = 2*math.pi*t/period
            headAngle = 0.59 * math.sin(x) - 0.0785
            hipAngle = 0.236 * math.sin(x) + 0.749
            kneeAngle = 0.82 * math.sin(x) + 0.73
            angles = [headAngle, -hipAngle, -hipAngle, kneeAngle, kneeAngle]
            SwingAPI.moveLimbs(motionProxy, limbs, angleNames, angles, 0.75, restInt)
            t += restInt

        angle = swingProxy.get_angle()
        if abs(angle) > maxAngle:
            maxAngle = angle
    return maxAngle

def stepPeriodicMax(motionProxy,swingProxy,period,numSwings):
    maxAngle = 0
    restInt=float(period)/2.0
    for x in range(0,numSwings):
        SwingAPI.position1Dynamic(motionProxy, 0.75)
        time.sleep(restInt)
        SwingAPI.position2Dynamic(motionProxy, 0.75)
        time.sleep(restInt)
        angle = swingProxy.get_angle()
        if abs(angle) > maxAngle:
            maxAngle = angle
    return maxAngle
def Main():
    period = 2.5
    maxAngle = 0
    maxPeriod = 0
    while period < 2.7:
        #IP, port and motion proxy
        robotIP="127.0.0.1"
        PORT=9559
        motionProxy=ALProxy("ALMotion",robotIP,PORT)
        swingProxy = SwingProxy.SwingProxy(robotIP,5005)
        print period
        angle = smoothPeriodicMax(motionProxy,swingProxy,period,12)
        print angle
        if angle > maxAngle:
            maxAngle = angle
            maxPeriod = period
        period += 0.01
        swingProxy.revert_world()
        time.sleep(15)

    print maxAngle
    print maxPeriod

Main()
