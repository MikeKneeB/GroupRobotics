import numpy as np


class Actor:
    def __init__(self, number_of_actions, td_errors, temperature_parameter):
        self.number_of_actions = number_of_actions
        self.td_errors = td_errors
        self.temperature_parameter = temperature_parameter
        self.policy = np.ones(td_errors.shape)/number_of_actions

    def update_policy(self, state, action, td_error):
        # update TD error of action in state
        self.td_errors[state + (action,)] += td_error
        # recalculate policy probabilities
        state_td_errors = self.td_errors[state]
        probabilities = softmax(state_td_errors, self.temperature_parameter)
        self.policy[state] = probabilities

    def get_next_action(self, state):
        probabilities = self.policy[state]
        return np.random.choice(self.number_of_actions, p=probabilities)


def softmax(state_td_errors, temperature_parameter):
    normalized = state_td_errors / temperature_parameter
    exponentiated = np.exp(normalized)
    sum_of_exponentiated = np.sum(exponentiated)
    return exponentiated / sum_of_exponentiated
