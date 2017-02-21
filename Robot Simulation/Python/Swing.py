import time
import math

LEGS_MIN=0
LEGS_MAX=1.2

class MyClass(GeneratedClass):
    def __init__(self):
        GeneratedClass.__init__(self)

    def onLoad(self):
        #put initialization code here
        pass

    def onUnload(self):
        #put clean-up code here
        pass

    def onInput_onStart(self):
        Main()
        pass

    def onInput_onStop(self):
        self.onUnload() #it is recommended to reuse the clean-up as the box is stopped
        self.onStopped() #activate the output of the box

def moveLimb(motionProxy,limb,angleNames,angles,speed):
    motionProxy.setStiffnesses(limb,1.0)
    motionProxy.setAngles(angleNames,angles,speed)
    time.sleep(0.001)
    motionProxy.setStiffnesses(limb,0.0)
    
def moveLimbs(motionProxy,limbs,angleNames,angles,speed):
    motionProxy.setStiffnesses(limbs,1.0)
    motionProxy.setAngles(angleNames,angles,speed)
    time.sleep(0.001)
    motionProxy.setStiffnesses(limbs,0.0)
    
def moveLegs(motionProxy,angle):
    limbs=["RLeg","LLeg"]
    angleNames=["RKneePitch","LKneePitch"]
    angles=[angle,angle]
    speed=0.5
    moveLimbs(motionProxy,limbs,angleNames,angles,speed)

def swingLegs(motionProxy,period,numSwings):
    numSteps=1000
    for i in range(0,numSwings):        
        for j in range(0,numSteps):
            x=j*(2*math.pi/numSteps)
            angle=(LEGS_MAX/2)+(LEGS_MAX/2)*math.sin(x/period)
            moveLegs(motionProxy,angle)
    
def Main():
    #IP, port and motion proxy
    robotIP="127.0.0.1"
    PORT=9559
    motionProxy=ALProxy("ALMotion",robotIP,PORT)
    
    swingLegs(motionProxy,1,10)
