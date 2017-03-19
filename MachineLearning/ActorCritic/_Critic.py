"""
Author: Harry Shaw
Date: 18/03/17
Stores and updates values using TD(0) algorithm.
"""

import numpy as np


class Critic:
    def __init__(self, state_dimensions, discount, value_learning_rate):
        self.learning_rate = value_learning_rate
        self.discount = discount

        self.values = np.zeros(state_dimensions)  #stores the values

    def get_td_error(self, previous_state, new_state, reward):
        """
        :return: the temporal difference error
        """
        v1 = self.values[new_state]
        v0 = self.values[previous_state]
        return reward + self.discount * v1 - v0

    def update_value(self, state, td_error):
        """
        Update the current estimate for the value of a state
        """
        self.values[state] += self.learning_rate * td_error
