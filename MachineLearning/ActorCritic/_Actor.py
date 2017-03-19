"""
Author: Harry Shaw
Date: 18/03/17
Stores the cumulative temporal difference errors and the policy
"""
import numpy as np


class Actor:
    def __init__(self, number_of_actions, state_dimensions, temperature_parameter):
        self.number_of_actions = number_of_actions
        policy_dimensions = state_dimensions + (number_of_actions,)

        self.td_errors = np.zeros(policy_dimensions)  # stores the cumulative TD errors for each state-action

        self.temperature_parameter = temperature_parameter
        self.policy = np.ones(policy_dimensions)/number_of_actions  # stores the policy for each state

    def update_policy(self, state, action, td_error):
        # update cumulative TD error of action in state
        self.td_errors[state + (action,)] += td_error

        # recalculate policy probabilities
        state_td_errors = self.td_errors[state]
        probabilities = softmax(state_td_errors, self.temperature_parameter)
        self.policy[state] = probabilities

    def get_next_action(self, state):
        """
        :return: a random action with a probability specified by the policy
        """
        probabilities = self.policy[state]
        return np.random.choice(self.number_of_actions, p=probabilities)


def softmax(state_td_errors, temperature_parameter):
    """
    Converts the cumulative temporal difference errors to a policy using softmax.

    :param state_td_errors: The temporal difference errors due to the actions in a state
    :return: The policy for that state
    """
    normalized = state_td_errors / temperature_parameter
    exponentiated = np.exp(normalized)
    sum_of_exponentiated = np.sum(exponentiated)
    return exponentiated / sum_of_exponentiated
