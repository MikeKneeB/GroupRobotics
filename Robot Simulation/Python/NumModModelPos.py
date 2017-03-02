import SwingAPI
import SwingProxy
import math
import time

from naoqi import ALProxy

def Main():
        robotIP = "127.0.0.1"
        robotPORT = 5995
        swingPORT = 5005
        motionProxy = ALProxy("ALMotion",robotIP,robotPORT)
        print "here"
        swingProxy = SwingProxy.SwingProxy(robotIP,swingPORT)


        prevAngle=0
        period = 1.5
        start=0
        toMove = False
        count=0

        while True:
                
                angle = swingProxy.get_angle()
                
                if (prevAngle*angle) < 0:
                        start=time.time()
                        toMove=True

                if(time.time()-start) > period/4 and toMove:
                        if angle > 0:
                                #SwingAPI.position1(motionProxy)
                                toMove = False
                                print 1
                        elif angle < 0 and toMove:
                                #SwingAPI.position2(motionProxy)
                                toMove = False
                                print 2

                prevAngle=angle

Main()
        
        
        
        
        
        

