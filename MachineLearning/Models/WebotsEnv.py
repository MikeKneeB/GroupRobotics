import time as Time

import threading

import math

import SwingProxy

from naoqi import ALProxy

import numpy as np


#for use with motionProxy kill methods
def performAction(motionProxy, action):

    #turns np.ndarray into float that naoqi needs
    action = action[0]
    #action = action[0]
    
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
    actionLimit = 0.5

    speed = abs(action) * actionLimit

    angles = [head, -hip, -hip, knee, knee]
    limbs = ["Head", "RLeg", "LLeg"]
    angleNames = ["HeadPitch", "RHipPitch", "LHipPitch", "RKneePitch", "LKneePitch"]
    #print "Sending Action: ", speed
    motionProxy.setStiffnesses(limbs, 1.0)
    motionProxy.setAngles(angleNames, angles, speed)

#performs action, and then 'dies'
class Controller(threading.Thread):
    def __init__(self, threadID, name, motionProxy, action):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.motionProxy = motionProxy
        self.action = action
    
    def run(self):
        performAction(self.motionProxy, self.action)
        Time.sleep(0.01)
        
        

class Dummy(object):

    def __init__(self, observation_space=4):
        self.shape = (observation_space,)


class WebotsEnv:
    
    def __init__(self):
        self.motionProxy = self.getNao()
        self.robotState = -1
        self.swingProxy = SwingProxy.SwingProxy("127.0.0.1",5005)
        self.observation_space = Dummy()
        self.lastTheta = -1
        self.lastOmega = -1
    
    def render(self):
        pass

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
	  
    def calculateReward(self, theta, omega):
        targetEnergy = (1 - np.cos(0.785)) * 9.81
        potentialEnergy = (1 - np.cos(theta)) * 9.81
        kineticEnergy = omega*omega*0.5
        currentEnergy = kineticEnergy + potentialEnergy
        reward = -0.1 * (targetEnergy - currentEnergy) * (targetEnergy - currentEnergy)
        return reward

    def botMoving(self, action):
        action = action[0]
        if action > 0:
            if self.getRobotState() < 0.9:
                return True
            else:
                return False
        else:
            if self.getRobotState() > 0.1:
                return True
            else:
                return False

    def step(self, action):
        reward = self.calculateReward(self.lastTheta, self.lastOmega)

        #performance = Controller(1, "Performance", self.motionProxy, action)
        #performance.start()
        performAction(self.motionProxy, action)
        recentTheta = self.lastTheta
        recentThetaTime = Time.time()
        #while action is being performed, get a recent theta for omega calculation
        
        while self.botMoving(action):
            #print self.getRobotState()
            recentTheta = self.swingProxy.getUpdate()
            recentThetaTime = Time.time()
            
        #sleep to make sure thetas are different
        Time.sleep(0.05)
        thetaState = self.swingProxy.getUpdate()
        thetaTime = Time.time()
        
        omegaState = (thetaState - recentTheta) / (thetaTime - recentThetaTime)
        #print "Recent Theta: ", recentTheta
        #print "Current Theta: ", thetaState
        #print "Omega: ", omegaState
        robotState = self.getRobotState()
        states = np.array([np.cos(thetaState), np.sin(thetaState), omegaState, robotState])
        self.lastTheta = thetaState
        self.lastOmega = omegaState

        #print "Obs: ", states
        #print "Reward: ", reward 
        return states, reward, False, {}

    def reset(self):
        #print "Slep"
        Time.sleep(1)
        #print "Woke"  
        
        thetaOne = self.swingProxy.getUpdate()
        time1 = Time.time()
        Time.sleep(1)
        thetaTwo = self.swingProxy.getUpdate()
        time2 = Time.time()
        robotState = self.getRobotState()

        omegaState = (thetaTwo - thetaOne)/(time2 - time1)
        #print "Got omega"
        #print "omega: ", omegaState
        thetaState = thetaTwo
        #print "Got theta"
        self.lastTheta = thetaState
        self.lastOmega = omegaState
        
        return np.array([np.cos(thetaState), np.sin(thetaState), omegaState, robotState])




def Main():
    print "Main started"

    wenv = WebotsEnv()
    
    print "resetting"
    wenv.reset()

    
    action = 0
    robotState = -1
    

    while 1:
        print "Waiting for action"
        action = float(raw_input("> "))
        
        if action == 99:
            return
        if action == 50:
            print robotState
            continue
        print "Making action"
        robotState = wenv.step(action)

if __name__ == '__main__':
    Main()
        
        
            

