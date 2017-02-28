import numpy as np
from naoqi import ALProxy

import SwingAPI
import SwingProxy

class Webot(object):

    def __init__(self, robot_ip, observe_port, action_port, target_theta, d_time=0.001):
        self.swing_proxy = SwingProxy.SwingProxy(robot_ip, observe_port)
        self.motion_proxy = ALProxy("ALMotion", robot_ip, action_port)

        self.g = 9.81

        self.pos_array = [0, 0]
        self.vel = 0

        self.target_theta = target_theta
        self.d_time = d_time

    def calc_reward(self, pos):

        energy = 0.5*self.vel*self.vel + self.g*np.cos(pos)

        target_energy = self.g*np.cos(pos)

        return -1 * (target_energy - energy) * (target_energy - energy)

    def step(self, action):
        if action == 0:
            pass
        elif action == 1:
            position1(self.motion_proxy)
        elif action == -1:
            position2(self.motion_proxy)

        pos = swing_proxy.getUpdate()

        del self.pos_array[0]

        self.pos_array.append(pos)

        self.vel = (self.pos_array[1] - self.pos_array[0])/self.d_time

        return np.array([np.cos(pos), np.sin(pos), self.vel]), self.calc_reward(pos), False, {}

    def reset(self):
        pos = swing_proxy.getUpdate()

        del self.pos_array[0]

        self.pos_array.append(pos)

        self.vel = (self.pos_array[1] - self.pos_array[0])/self.d_time

        return np.array([np.cos(pos), np.sin(pos), self.vel])
