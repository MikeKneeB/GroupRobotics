import numpy as np

class SARSA:
    def __init__(self, number_of_actions, state_dimensions, discount, value_learning_rate, policy_update_rate):
        self.numberOfActions = number_of_actions
        self.stateDimensions = state_dimensions
        self.discount = discount
        self.valueLearningRate = value_learning_rate
        self.policyUpdateRate = policy_update_rate

    """def update_policy(self, state, action, td_error):
        # update TD error of action in state
        self.td_errors[state + (action,)] += self.policy_update_rate * td_error
        # recalculate policy probabilities
        state_td_errors = self.td_errors[state]
        probabilities = softmax(state_td_errors)
        self.policy[state] = probabilities"""

    def get_next_action(self, state):
        probabilities = self.policy[state]
        return np.random.choice(self.number_of_actions, p=probabilities)


def softmax(state_q_values):
    # scale TD errors so the mean is 0 with an absolute maximum of 2
    normalized = state_q_values/0.5

    exponentiated = np.exp(normalized)
    sum_of_exponentiated = np.sum(exponentiated)
    return exponentiated / sum_of_exponentiated