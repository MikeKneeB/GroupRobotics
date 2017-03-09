import time

hipRange = 0.472
kneeRange = 1.64
headRange = 1.18

def moveLimbs(motionProxy,limbs,angleNames,angles,speed,restInt):
    motionProxy.setStiffnesses(limbs,1.0)
    motionProxy.setAngles(angleNames,angles,speed)
    time.sleep(restInt)

def position1Mono(motionProxy, speed):
    #Limbs and angle names
    limbs=["Head","RLeg","LLeg"]
    angleNames=["HeadPitch","RHipPitch","LHipPitch","RKneePitch","LKneePitch"]
    #Angle values
    headAngle=-0.6685
    hipAngle=0.513
    kneeAngle=-0.09
    angles=[headAngle,-hipAngle,-hipAngle,kneeAngle,kneeAngle]
    #Rest interval
    restInt=0.001
    #Send the move request
    moveLimbs(motionProxy,limbs,angleNames,angles,speed,restInt)


def position2Mono(motionProxy, speed):
    #Limbs and angle names
    limbs=["Head","RLeg","LLeg"]
    angleNames=["HeadPitch","RHipPitch","LHipPitch","RKneePitch","LKneePitch"]
    #Angle values
    headAngle=0.5115
    hipAngle=0.985
    kneeAngle=1.55
    angles=[headAngle,-hipAngle,-hipAngle,kneeAngle,kneeAngle]
    #Rest interval
    restInt=0.001
    #Send the move request
    moveLimbs(motionProxy,limbs,angleNames,angles,speed,restInt)

def position1Dynamic(motionProxy,kneeSpeed):
    restInt = 0.05
    #Hip
    hipLimbs = ["RLeg", "LLeg"]
    hipAngleNames = ["RHipPitch", "LHipPitch"]
    hipAngles = [-0.513, -0.513]
    hipSpeed = kneeSpeed*(hipRange/kneeRange)
    #Knees
    kneeLimbs = hipLimbs
    kneeAngleNames = ["RKneePitch", "LKneePitch"]
    kneeAngles = [-0.09,-0.09]
    #Head
    headLimbs = ["Head"]
    headAngleNames = ["HeadPitch"]
    headAngle = [-0.6685]
    headSpeed = kneeSpeed*(headRange/kneeRange)

    moveLimbs(motionProxy, kneeLimbs, kneeAngleNames, kneeAngles, kneeSpeed, restInt)
    moveLimbs(motionProxy, headLimbs, headAngleNames, headAngle, headSpeed, restInt)
    moveLimbs(motionProxy, hipLimbs, hipAngleNames, hipAngles, hipSpeed, restInt)


def position2Dynamic(motionProxy,kneeSpeed):
    restInt = 0.05
    #Hip
    hipLimbs = ["RLeg", "LLeg"]
    hipAngleNames = ["RHipPitch", "LHipPitch"]
    hipAngles = [-0.985, -0.985]
    hipSpeed = kneeSpeed*(hipRange/kneeRange)
    #Knees
    kneeLimbs = hipLimbs
    kneeAngleNames = ["RKneePitch", "LKneePitch"]
    kneeAngles = [1.55, 1.55]
    #Head
    headLimbs = ["Head"]
    headAngleNames = ["HeadPitch"]
    headAngle = [0.5115]
    headSpeed = kneeSpeed*(headRange/kneeRange)

    moveLimbs(motionProxy, kneeLimbs, kneeAngleNames, kneeAngles, kneeSpeed, restInt)
    moveLimbs(motionProxy, headLimbs, headAngleNames, headAngle, headSpeed, restInt)
    moveLimbs(motionProxy, hipLimbs, hipAngleNames, hipAngles, hipSpeed, restInt)

