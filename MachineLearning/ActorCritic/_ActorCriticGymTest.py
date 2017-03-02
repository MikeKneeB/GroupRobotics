import random

import gym
import numpy as np

import time

import ActorCritic as ac

# global variables
THETAS = 20
OMEGAS = 17
ACTIONS = 9


def rescale(observations):
    x, y, omega_raw = observations
    theta_raw = np.arctan(x / y)
    theta = int(round((theta_raw + np.pi) * THETAS / (2 * np.pi)))
    if theta == THETAS:
        theta = 0
    omega = int(round(omega_raw + 8) * (OMEGAS - 1) / 16)
    return theta, omega


def torque_from(action):
    return np.array((action * 4.0 / ACTIONS - 2,))


def get_observations(torque, env):
    observations = env.step(torque)
    positions, reward, failed, info = observations
    theta, omega = rescale(positions)

    return (theta, omega), reward


# resets enviroment to random position and returns the initial position and velocity
def reset_enviroment(env):
    observations = env.reset()
    theta, omega = rescale(observations)
    return theta, omega


def main():
    env = gym.make('Pendulum-v0')
    env.reset()
    f = open('gym.txt', 'w')

    # (positions, velocities)
    state_dimensions = (THETAS, OMEGAS)

    discount = 0.8

    actor = ac.ActorCritic(ACTIONS, state_dimensions, discount, value_learning_rate=0.5, policy_update_rate=0.1)

    f.write('epoch\treward\n')
    epochs = 10000
    exploration = epochs / 2
    display = 9 * epochs / 10
    for epoch in range(epochs):
        # reset environment and extract initial state
        state = reset_enviroment(env)
        cumulativeReward = 0
        print epoch
        for step in range(300):
            if epoch > exploration:
                action = actor.get_next_action(state)
            else:
                action = random.randint(0, ACTIONS-1)
            torque = torque_from(action)

            # perform action on environment
            new_state, reward = get_observations(torque, env)
            cumulativeReward += reward

            # critique the quality of the action
            actor.critique(state, action, new_state, reward)
            state = new_state

            # print display, ",", epochs
            # if epoch > display:
            #     env.render()

        f.write('{}\t{}\n'.format(epoch, cumulativeReward))
    f.close()


if __name__ == '__main__':
    main()
