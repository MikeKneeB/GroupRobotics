import numpy as np

class SARSA:
    def __init__(self, number_of_actions, state_dimensions, discount, learning_rate, temperature_parameter):
        self.numberOfActions = number_of_actions                    #takes tuple as argument
        self.stateDimensions = state_dimensions                     #takes tuple as argument
        self.QDimensions = state_dimensions + (number_of_actions,)     #added tuples create an array
        self.discount = discount
        self.learningRate = learning_rate
        self.policy = np.ones(self.QDimensions)/number_of_actions
        self.QValues = np.zeros(self.QDimensions)
        self.temperatureParameter = temperature_parameter


    """def update_policy(self, state, action, td_error):
        # update TD error of action in state
        self.td_errors[state + (action,)] += self.policy_update_rate * td_error

        # recalculate policy probabilities
        state_td_errors = self.td_errors[state]
        probabilities = softmax(state_td_errors)
        self.policy[state] = probabilities"""

    def update_Policy(self, state, action, new_state, new_action, reward):
        #updates Q value for a given state-action transition
        #make this easier to read
        self.QValues[state, action] = self.QValues[state, action] + self.learningRate * (reward + self.discount * self.QValues[new_state, new_action] - self.QValues[state, action])

        self.policy[state]= softmax(self.QValues[state], self.temperatureParameter)


    def get_next_action(self, state):
        probabilities = self.policy[state]
        return np.random.choice(self.numberOfActions, p=probabilities)

#Directly from ActorCritic. Makes a normalised probability distribution
def softmax(state_td_errors, temperature_parameter):
    normalized = state_td_errors / temperature_parameter
    exponentiated = np.exp(normalized)
    sum_of_exponentiated = np.sum(exponentiated)
    return exponentiated / sum_of_exponentiated