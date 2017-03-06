import time

import threading

import math

from naoqi import ALProxy

#sets up connection to Nao in Webots, returns proxy for motion

def getNao():

    #IP, port and motion proxy

    robotIP="127.0.0.1"

    PORT=9559

    motionProxy=ALProxy("ALMotion",robotIP,PORT)

    return motionProxy

#for use with motionProxy kill methods
def performAction(motionProxy, action):
    
    #set direction of motion and make action non-zero
    
    #else
    head = -0.6685
    hip = 0.513
    knee = -0.09

    if action > 0:
        head = 0.5115
        hip = 0.985
        knee = 1.55
    elif action == 0:
        action = 0.00001

    # action speed modifier, to be changed to match real bot limits
    actionLimit = 1

    speed = abs(action) * actionLimit

    angles = [head, -hip, -hip, knee, knee]
    limbs = ["Head", "RLeg", "LLeg"]
    angleNames = ["HeadPitch", "RHipPitch", "LHipPitch", "RKneePitch", "LKneePitch"]

    motionProxy.setStiffnesses(limbs, 1.0)
    motionProxy.setAngles(angleNames, angles, speed)

#experiment in threading requries exitFlag and action public vars
class Controller(threading.Thread):
    def __init__(self, threadID, name, motionProxy, action):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.motionProxy = motionProxy
        self.action = action
    
    def run(self):
        performAction(self.motionProxy, self.action)

#takes in robot angle, spits out state value in range [0.0 to 1.0]
#uses knee as knee take the longest to finish motion
def getRobotState(kneeAngle):
    kneeRange = 1.64
    robotState = (kneeAngle + 0.09)/ kneeRange
    return robotState


def Main():
    print "Main started"

    motionProxy = getNao()

    action = 0
    

    while 1:
        action = float(raw_input("> "))
        motionProxy.killAll()
        if action == 99:
            return
        if action == 50:
            angleNames = ["HeadPitch", "RHipPitch", "LHipPitch", "RKneePitch", "LKneePitch"]
            angleValues = motionProxy.getAngles(angleNames, False)
            robotState = getRobotState(angleValues[4])
            print robotState
            action = 0

        control = Controller(1, "Controller", motionProxy, action)
        control.start()

if __name__ == '__main__':
    Main()
        
        
            
