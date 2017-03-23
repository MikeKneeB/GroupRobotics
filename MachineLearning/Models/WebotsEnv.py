"""
WebotsEnvironment.py

Provides an OpenAI Gym like environment for agent communication with the Webots swing simulation.

Author: Henry Gaskin

Date Published: 23/03/17

Contact: hrg369@student.bham.ac.uk

"""

import time as Time

import math

import SwingProxy

from naoqi import ALProxy

import numpy as np


#function similar to that of motion in the SwingAPI class. Functions should be interchangable
#previously used within a threading class so currently removed from WebotsEnv class.
def performAction(motionProxy, action):

    #DQN wraps action in an array, DDPG wraps it in two arrays.
    action = action[0]
    #action = action[0]
    
    #action is assumed to be either 1 or -1
    
    #set motion destination angles
    #else
    head = -0.6685
    hip = 0.513
    knee = -0.09

    if action > 0:
        head = 0.5115
        hip = 0.985
        knee = 1.55

    # action speed modifier, prevents simulated robot from going to fast and breaking the system
    actionLimit = 0.75
    
    #speed of limbs
    speed = abs(action) * actionLimit

    #data needed for NAOqi setStiffness and setAngles methods
    angles = [head, -hip, -hip, knee, knee]
    limbs = ["Head", "RLeg", "LLeg"]
    angleNames = ["HeadPitch", "RHipPitch", "LHipPitch", "RKneePitch", "LKneePitch"]
    
    #move limbs
    motionProxy.setStiffnesses(limbs, 1.0)
    motionProxy.setAngles(angleNames, angles, speed)        

#provides an emulation of an OpenAI environment for swift transition
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
        self.lastAction = 0
        self.output = open("WebotsEnvOutput.txt","w")
        self.started = False
        self.startTime = -1
    
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
    #if dynamicMotion was used, any limb action could be used
    def getRobotState(self):
        angleNames = ["HeadPitch", "RHipPitch", "LHipPitch", "RKneePitch", "LKneePitch"]
        angleValues = self.motionProxy.getAngles(angleNames, False)
        kneeAngle = angleValues[4]
        kneeRange = 1.64
        self.robotState = (kneeAngle + 0.09)/ kneeRange
        return self.robotState
	
    #reward is calculated using the same function as the dumbbell environment
    def calculateReward(self, theta, omega):
        targetEnergy = (1 - np.cos(0.785)) * 9.81
        potentialEnergy = (1 - np.cos(theta)) * 9.81
        kineticEnergy = omega*omega*0.5
        currentEnergy = kineticEnergy + potentialEnergy
        reward = -0.1 * (targetEnergy - currentEnergy) * (targetEnergy - currentEnergy)
        return reward

    #returns true if the robot is still making a transition from one motion to the other.
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
    
    #function called by agent to send action and supply observation space
    def step(self, action):
        #get reward from previous action and state
        reward = self.calculateReward(self.lastTheta, self.lastOmega)
        
        #perform new action as dictated from the agent
        performAction(self.motionProxy, action)
        
        recentTheta = self.lastTheta
        recentThetaTime = Time.time()
        
        #if the robot isn't going to move, sleep instead to normalise action length
        if action == self.lastAction:
            Time.sleep(0.4)
            
        #wait until the bot has finished moving, will take roughly ~0.4s
        while self.botMoving(action):
            pass
     
        #collect two positions and the time between them to calculate the speed
        recentTheta = self.swingProxy.get_angle()
        recentThetaTime = Time.time()       
        #sleep to make sure thetas are different
        Time.sleep(0.05)
        thetaState = self.swingProxy.get_angle()
        thetaTime = Time.time()
        omegaState = (thetaState - recentTheta) / (thetaTime - recentThetaTime)
        
        #get the current state of the robot, either 1 or -1
        robotState = self.getRobotState()
        
        states = np.array([np.cos(thetaState), np.sin(thetaState), omegaState, robotState])
        
        #update memory used for reward calculations and logic
        self.lastTheta = thetaState
        self.lastOmega = omegaState
        self.lastAction = action

        return states, reward, False, {}

    def reset(self):
        
        #start global timer for debug use
        if self.started == False:
            self.started = True
            self.startTime = Time.time()
            
        #make initial measurements of environment state
        thetaOne = self.swingProxy.get_angle()
        time1 = Time.time()
        Time.sleep(0.05)
        thetaTwo = self.swingProxy.get_angle()
        time2 = Time.time()
        robotState = self.getRobotState()
        omegaState = (thetaTwo - thetaOne)/(time2 - time1)
        thetaState = thetaTwo
        self.lastTheta = thetaState
        self.lastOmega = omegaState
        
        #return statespace
        return np.array([np.cos(thetaState), np.sin(thetaState), omegaState, robotState])



#debug main used for manual input of actions
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

#prevents main being called
if __name__ == '__main__':
    Main()
        
        
            

