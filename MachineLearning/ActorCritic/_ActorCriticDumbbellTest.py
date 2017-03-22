"""
Author: Harry Shaw
Date: 18/03/17

Test of actor-critic on dumbbell pendulum model
"""

import time

from MachineLearning.Models import DumbellEnvironment as de


import ActorCritic as ac
import numpy as np

np.seterr(all='raise')  # If numpy encounters a NaN or something, want it to raise exceptions

# global variables
Y = 179
X = 179
OMEGAS = 57
ACTIONS = 29


# Rescale observations so they are integers from 0
def rescale(observations):
    x_raw, y_raw, omega_raw = observations
    y = int(round((y_raw + 1) * (Y - 1) / 2))
    x = int(round((x_raw + 1) * (X - 1) / 2))
    omega = int(round(omega_raw + 3) * (OMEGAS - 1) / 6)
    # print "Observations: ", observations
    # print "Rescaled: {}, {}, {}".format(x, y, omega)
    return x, y, omega


# From an integer action, get the corresponding pendulum torque
def torque_from(action):
    out = action * 10 / (ACTIONS - 1) - 5
    # print "Action: ", action
    # print "Out: ", out
    return out


# From an integer action, get the corresponding pendulum torque
def get_observations(torque, env):
    observations = env.step(torque)
    positions, reward, done, info = observations
    # print("Positions: ", positions)
    x, y, omega = rescale(positions)
    return (x, y, omega), reward


# resets environment to random position and returns the initial position and velocity
def reset_environment(env):
    observations = env.reset()
    x, y, omega = rescale(observations)
    return x, y, omega


def main():
    env = de.Dumbell()
    discounts = [0.6, 0.8, 0.9, 0.95, 0.99]
    for discount in [0.95]:
        print "Discount: ", discount
        env.reset()
        filename = 'dblol{}.txt'.format(discount)

        # outputs epoch and cumulative reward gained during an epoch
        f = open(filename, 'w')
        env.reset()

        # (x position, y position, velocity)
        state_dimensions = (Y, X, OMEGAS)

        # If temperature_parameter is too low, you will get NaN errors
        actor_critic = ac.ActorCritic(ACTIONS, state_dimensions, discount=discount, learning_rate=1,
                                      temperature_parameter=500)

        epochs = 5000
        display = 5000  # epoch after which environment renders
        for epoch in range(epochs):
            print epoch

            # reset environment and extract initial state
            state = reset_environment(env)

            cumulative_reward = 0
            for step in range(300):

                action = actor_critic.get_next_action(state)

                # perform action on environment
                torque = torque_from(action)
                new_state, reward = get_observations(torque, env)
                reward = float(reward)

                # critique the quality of the action
                actor_critic.critique(state, action, new_state, reward)

                cumulative_reward += reward
                state = new_state

                if epoch > display:
                    time.sleep(0.5)
                    env.render()

            f.write('{}\t{}\n'.format(epoch, cumulative_reward))
        f.close()


if __name__ == '__main__':
    main()
