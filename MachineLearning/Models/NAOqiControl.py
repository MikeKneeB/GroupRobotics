import time

import math

from naoqi import ALProxy



#sets up connection to Nao in Webots, returns proxy for motion

def getNao():

    #IP, port and motion proxy

    robotIP="127.0.0.1"

    PORT=9559

    motionProxy=ALProxy("ALMotion",robotIP,PORT)

    return motionProxy



#takes in angles and speed{0 to 1.0}, returns when motion has been made

def moveNaoToAngle(motionProxy, head, hip, knee, speed):



    #used in setStiffness

    limbs=["Head","RLeg","LLeg"]

    #used in setAngle 

    angleNames=["HeadPitch","RHipPitch","LHipPitch","RKneePitch","LKneePitch"]

        

    angles=[head,-hip,-hip,knee,knee]

    

    #talk to nao

    motionProxy.setStiffnesses(limbs,1.0)

    motionProxy.setAngles(angleNames,angles,speed)



#performs a small motion by action{-1.0 to 1.0} takes in current angle, spits out new one

def makeAction(motionProxy, head, hip, knee, action):

    

    #limit action

    if action > 1.0:

        action = 1.0

    elif action < -1.0:

        action = -1.0

    

    #also need a minimum action/speed again this needs tuning

    #if action < minAct...

    

    #number of discrete angles used for calculation, needs tuning/testing 

    increments = 1000        

    

    headAngleRange = 1.18

    hipAngleRange = 0.472

    kneeAngleRange = 1.64

    

    headIncrement = headAngleRange / increments

    hipIncrement = hipAngleRange / increments

    kneeIncrement = kneeAngleRange / increments

    

    if action < 0:

        headIncrement = -headIncrement

        hipIncrement = -hipIncrement

        kneeIncrement = -kneeIncrement

    elif action == 0:

        headIncrement = 0

        hipIncrement = 0

        kneeIncrement = 0

    

    head += headIncrement

    hip += headIncrement

    knee += kneeIncrement

    

    speed = abs(action)

    

    #Min Angle values: Nao "Planking" position1

    minHeadAngle=-0.6685

    minHipAngle=0.513

    minKneeAngle=-0.09

    

    #Max angle values: Nao "Balling" position2

    maxHeadAngle=0.5115

    maxHipAngle=0.985

    maxKneeAngle=1.55    



    #limit angles    

    if head < minHeadAngle:

        head = minHeadAngle

    elif head > maxHeadAngle:

        head = maxHeadAngle

    if hip < minHipAngle:

        hip = minHipAngle

    elif hip > maxHipAngle:

        hip = maxHipAngle

    if knee < minKneeAngle:

        knee = minKneeAngle

    elif knee > maxKneeAngle:

        knee = maxKneeAngle

    

    moveNaoToAngle(motionProxy, head, hip, knee, speed)

    return head, hip, knee

    

if __name__ == '__main__':

    

    motionProxy = getNao()

    

    

    # reset Nao to position2

    head = 0.5115

    hip = 0.985

    knee = 1.55

    moveNaoToAngle(motionProxy, head, hip, knee, 1.0)

    

    # should swing legs up to position1 (if it works)

    for _ in range(1000):

        head, hip, knee = makeAction(motionProxy, head, hip, knee, -0.5)
