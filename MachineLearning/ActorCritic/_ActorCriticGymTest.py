import random

import gym
import numpy as np
import ActorCritic as ac

np.seterr(all='raise')

# global variables
THETAS = 20
OMEGAS = 17
ACTIONS = 9


def rescale(observations, g):
    x, y, omega_raw = observations
    theta_raw = np.arctan(x / y)
    theta = int(round((theta_raw + np.pi/2) * THETAS / np.pi))
    if theta == THETAS:
        theta = 0
    omega = int(round(omega_raw + 8) * (OMEGAS - 1) / 16)
    if g is not None:
        g.write("{}\t{}\n".format(theta, omega))
    return theta, omega


def torque_from(action):
    return np.array((action * 4.0 / (ACTIONS - 1) - 2,))


def get_observations(torque, env, g=None):
    observations = env.step(torque)
    positions, reward, failed, info = observations
    theta, omega = rescale(positions, g)

    return (theta, omega), reward


# resets enviroment to random position and returns the initial position and velocity
def reset_enviroment(env, g=None):
    observations = env.reset()
    theta, omega = rescale(observations, g)
    return theta, omega


def main():
    env = gym.make('Pendulum-v0')
    env.reset()
    f = open('gym.txt', 'w')
    g = None

    # (positions, velocities)
    state_dimensions = (THETAS, OMEGAS)

    discount = 0.8

    actor_critic = ac.ActorCritic(ACTIONS, state_dimensions, discount, value_learning_rate=0.5, policy_update_rate=0.5)

    f.write('epoch\treward\n')
    epochs = 200000
    exploration = epochs / 4
    display = 19 * epochs / 20
    for epoch in range(epochs):
        # reset environment and extract initial state
        state = reset_enviroment(env, g)
        cumulativeReward = 0
        print epoch
        for step in range(300):
            # if epoch > exploration:
            #     action = actor_critic.get_next_action(state)
            # else:
            #     action = random.randint(0, ACTIONS - 1)

            action = actor_critic.get_next_action(state)

            torque = torque_from(action)

            # perform action on environment
            new_state, reward = get_observations(torque, env, g)
            cumulativeReward += reward

            # critique the quality of the action
            actor_critic.critique(state, action, new_state, reward)
            state = new_state

            # print display, ",", epochs
            # if epoch > display:
            #     env.render()
        actor_critic.actor.temperature_parameter = np.abs(actor_critic.actor.td_errors).max() / 3

        f.write('{}\t{}\n'.format(epoch, cumulativeReward))
    f.close()
    np.savetxt('gym_values', actor_critic.critic.values)


if __name__ == '__main__':
    main()
