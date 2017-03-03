import time
import SwingAPI
import SwingProxy
from naoqi import ALProxy

def Main():
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

    #Natural period of the real swing
    #period = 2.561

    #Initialise variables
    prevAngle = 0
    start = 0
    toMove = False

    #Loop
    while True:
        #Get the next angle
        angle = swingProxy.get_angle()
        #If negative then just passed bottom of swing
        if (prevAngle*angle) < 0:
            #Set time passed bottom of swing
            start = time.time()
            toMove = True
        #Change position at estimated top of swing and if not in correct pos
        if (time.time()-start) > (period/4)-0.1 and toMove:
            #Decide which position to switch to
            if angle > 0:
                SwingAPI.position1(motionProxy)
                toMove = False
            elif angle < 0:
                SwingAPI.position2(motionProxy)
                toMove = False
        #Set prevAngle for next iteration
        prevAngle = angle
Main()
