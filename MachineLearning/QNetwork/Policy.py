"""
Author: Chris Patmore
Date: 22/03/2017
Description: Policy for action selection code,
credit to Matthias plappert, for inspiration
https://github.com/matthiasplappert/keras-rl/blob/master/rl/policy.py
"""
import numpy as np

# action selection policy
# action is selected based using a boltzmann distribution based on the
# Q-value of the state action pair.
# i.e. if tau -> 0 the best action will always be selected, however this will
# start clipping and break selection, suggested min tau is 1


class BoltzmannQPolicy(object):
    def __init__(self, tau=1., clip=(-1000., 1000.)):
        super(BoltzmannQPolicy, self).__init__()
        self.tau = tau
        self.clip = clip

    # select and action using the boltzmann distribution
    def select_action(self, q_values):
        assert q_values.ndim == 1
        q_values = q_values.astype('float64')
        nb_actions = q_values.shape[0]

        exp_values = np.exp(np.clip(q_values / self.tau, self.clip[0], self.clip[1]))
        probs = exp_values / np.sum(exp_values)
        action = np.random.choice(range(nb_actions), p=probs)
        return action

    # set tau, larger tau -> flatter distribution -> more random selection
    def set_tau(self, new_tau):
        self.tau = new_tau
