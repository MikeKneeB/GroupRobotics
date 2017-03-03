import time

def moveLimbs(motionProxy,limbs,angleNames,angles,speed,restInt):
    motionProxy.setStiffnesses(limbs,1.0)
    motionProxy.setAngles(angleNames,angles,speed)
    time.sleep(restInt)

def position1(motionProxy):
    #Limbs and angle names
    limbs=["Head","RLeg","LLeg"]
    angleNames=["HeadPitch","RHipPitch","LHipPitch","RKneePitch","LKneePitch"]
    #Angle values
    headAngle=-0.6685
    hipAngle=0.513
    kneeAngle=-0.09
    angles=[headAngle,-hipAngle,-hipAngle,kneeAngle,kneeAngle]
    #speed and rest interval
    speed=0.75
    restInt=0.001
    #Send the move request
    moveLimbs(motionProxy,limbs,angleNames,angles,speed,restInt)


def position2(motionProxy):
    #Limbs and angle names
    limbs=["Head","RLeg","LLeg"]
    angleNames=["HeadPitch","RHipPitch","LHipPitch","RKneePitch","LKneePitch"]
    #Angle values
    headAngle=0.5115
    hipAngle=0.985
    kneeAngle=1.55
    angles=[headAngle,-hipAngle,-hipAngle,kneeAngle,kneeAngle]
    #speed and rest interval
    speed=0.75
    restInt=0.001
    #Send the move request
    moveLimbs(motionProxy,limbs,angleNames,angles,speed,restInt)
