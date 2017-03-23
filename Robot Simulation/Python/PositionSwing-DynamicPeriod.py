import time
import SwingAPI
import SwingProxy
from naoqi import ALProxy

#   Author: Harry Withers
#   Date:   09/03/2017
#
#   This file does the same as PositionSwing.py but now the robot
#   dynamically calculates the period of the swing to improve its
#   drive.

def Main():
    #Open file for data
    f = open("PositionSwingDynamic.txt","w")

    #Address of Robot and Swing
    IP = "127.0.0.1"
    ROBOT_PORT = 9559
    SWING_PORT = 5005
    #Robot and Swing proxies
    motionProxy = ALProxy("ALMotion", IP, ROBOT_PORT)
    #Change this line to connect to real angle encoder
    swingProxy = SwingProxy.SwingProxy(IP, SWING_PORT)

    #Natural period of the virtual swing
    period = 3.27
    #Delay after period/4 to swing
    #Negative if before period/4
    delay = -0.1
    #Natural period of the real swing
    #period = 2.561

    #First previous angle
    prevAngle = 0
    #Initial start time for swinging
    start = 0
    #Does not need to move yet
    toMove = False
    #Start time of simulation
    startSim = time.time()
    #Variables to calculate period
    skip = True
    periodStart = startSim
    count = 1

    #Loop
    while True:
        #Offset for if 0 is not centre of swing
        offset = 0
        #Get the next angle
        angle = swingProxy.get_angle()+offset
        #If negative then just passed bottom of swing
        if (prevAngle*angle) < 0:
            #Set time passed bottom of swing
            start = time.time()
            toMove = True
            #Calculate period
            if skip:
                skip = False
            elif not skip:
                count += 1
                temp = time.time()-periodStart
                print temp
                period = ((temp)+(period*(count-1)))/count
                periodStart = start
                skip = True
        #Change position at estimated top of swing and if not in correct pos
        if (time.time()-start) > (period/4)+delay and toMove:
            #Decide which position to switch to
            if angle > 0:
                SwingAPI.position1Dynamic(motionProxy,0.75)
                toMove = False
            elif angle < 0:
                SwingAPI.position2Dynamic(motionProxy,0.75)
                toMove = False
        #Set prevAngle for next iteration
        prevAngle = angle
        # Find time elapsed since start of simulation
        timeElaps = time.time() - startSim
        #Stringify time elapsed and angle
        stimeElaps = str(timeElaps)
        sangle = str(angle)
        # print values to file with tabs and new line
        f.write(stimeElaps)
        f.write('\t\t')
        f.write(sangle)
        f.write('\n')
Main()
