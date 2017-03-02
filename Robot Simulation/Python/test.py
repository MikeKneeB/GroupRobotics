from naoqi import ALProxy

import SwingAPI

robotIP = "127.0.0.1"
PORT1=9559
PORT2=5115

proxy1=ALProxy("ALMotion",robotIP,PORT1)
proxy2=ALProxy("ALMotion",robotIP,PORT2)

SwingAPI.position1(proxy1)
SwingAPI.position2(proxy2)
