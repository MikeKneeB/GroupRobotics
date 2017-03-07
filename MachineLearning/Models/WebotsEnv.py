import time

import threading

import math

from naoqi import ALProxy

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

class Controller(threading.Thread):
    def __init__(self, threadID, name, motionProxy, action):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.motionProxy = motionProxy
        self.action = action
    
    def run(self):
        performAction(self.motionProxy, self.action)

class WebotsEnv:
    
    def __init__(self):
        self.motionProxy = self.getNao()
        self.robotState = -1
        
        #sets up connection to Nao in Webots, returns proxy for motion
    def getNao(self):

        #IP, port and motion proxy

        robotIP="127.0.0.1"

        PORT=9559

        motionProxy=ALProxy("ALMotion",robotIP,PORT)

        return motionProxy

    #takes in robot angle, spits out state value in range [0.0 to 1.0]
    #uses knee as knee take the longest to finish motion
    def getRobotState(self):
        angleNames = ["HeadPitch", "RHipPitch", "LHipPitch", "RKneePitch", "LKneePitch"]
        angleValues = self.motionProxy.getAngles(angleNames, False)
        kneeAngle = angleValues[4]
        kneeRange = 1.64
        self.robotState = (kneeAngle + 0.09)/ kneeRange
        return self.robotState

    #submethod of step(), gets state and gives action 
    def robotStep(self, action):
        self.motionProxy.killAll()
        control = Controller(1, "Controller", self.motionProxy, action)
        control.start()
    
        #I assume we should wait a bit for time to pass
        #so that robot is in a new state, hopefully due
        #to the action inputted. Whether in this method
        #or somewhere else, who knows.
    
        #time.sleep(1)

        return self.getRobotState





def Main():
    print "Main started"

    wenv = WebotsEnv()

    action = 0
    robotState = -1
    

    while 1:
        action = float(raw_input("> "))
        
        if action == 99:
            return
        if action == 50:
            print wenv.getRobotState()
            continue

        robotState = wenv.robotStep(action)

if __name__ == '__main__':
    Main()
        
        
            
