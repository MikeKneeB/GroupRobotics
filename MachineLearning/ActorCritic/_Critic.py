import numpy as np
from math import isnan


class Critic:
    def __init__(self, state_dimensions, discount, value_learning_rate):
        self.learning_rate = value_learning_rate
        self.discount = discount

        self.values = np.zeros(state_dimensions)

    def get_td_error(self, previous_state, new_state, reward):
        # print "Reward of new state: ", r1
        v1 = self.values[new_state]
        # print "Value of new state: ", v1
        v0 = self.values[previous_state]
        # print "Value of previous state: ", v0
        return reward + self.discount * v1 - v0

    def update_value(self, state, td_error):
        self.values[state] += self.learning_rate * td_error
