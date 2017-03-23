"""
Author: Jay Morris
Date: 18/03/17

Complete SARSA learning system. To use, use get_next_action to have a reference to two consecutive state-action pairs,
 then use these as the input for update_policy, then repeat.
"""

import numpy as np

#
class SARSA:
    def __init__(self, number_of_actions, state_dimensions, discount, learning_rate, temperature_parameter):
        self.numberOfActions = number_of_actions                    #takes tuple as argument
        self.stateDimensions = state_dimensions                     #integer to be cast as tuple
        self.QDimensions = state_dimensions + (number_of_actions,)     #added tuples create an array
        self.discount = discount
        self.learningRate = learning_rate
        self.policy = np.ones(self.QDimensions)/number_of_actions
        self.QValues = np.zeros(self.QDimensions)
        self.temperatureParameter = temperature_parameter



    def update_Policy(self, state, action, new_state, new_action, reward):
        """
        :param state: the previous state of the agent
        :param action: the previous action taken by the agent
        :param new_state: the current state of the agent
        :param new_action: the current action to be taken by the agent
        :param reward: the reward of the state the agent will move into

        updates Q value for a given state-action transition, then changes the policy based on this
        """

        state_action = state + (action,)
        new_state_action = new_state + (new_action,)
        difference = reward + self.discount * self.QValues[new_state_action] - self.QValues[state_action]
        self.QValues[state_action] += self.learningRate * difference

        self.policy[state]= softmax(self.QValues[state], self.temperatureParameter)


    def get_next_action(self, state):
        """
        :param state: The state to calculate the next action from
        :return: the index of the action to take
        """
        probabilities = self.policy[state]
        return np.random.choice(self.numberOfActions, p=probabilities)

#Directly from ActorCritic. Makes a normalised probability distribution
def softmax(state_td_errors, temperature_parameter):
    normalized = state_td_errors / temperature_parameter
    exponentiated = np.exp(normalized)
    sum_of_exponentiated = np.sum(exponentiated)
    return exponentiated / sum_of_exponentiated