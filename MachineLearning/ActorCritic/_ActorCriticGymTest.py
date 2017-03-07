import random

import gym
import numpy as np

import ActorCritic as ac

np.seterr(all='raise')  # If numpy encounters a NaN or something, want it to raise exceptions

# global variables
X = 9
Y = 9
OMEGAS = 17
ACTIONS = 9


# Rescale observations so they are integers from 0
def rescale(observations):
    x_raw, y_raw, omega_raw = observations
    x = int(round((x_raw + 1) * (X - 1) / 2))
    y = int(round((y_raw + 1) * (Y - 1) / 2))
    omega = int(round(omega_raw + 8) * (OMEGAS - 1) / 16)
    return x, y, omega


# From an integer action, get the corresponding pendulum torque
def torque_from(action):
    return np.array((action * 4.0 / (ACTIONS - 1) - 2,))


# From an integer action, get the corresponding pendulum torque
def get_observations(torque, env):
    observations = env.step(torque)
    positions, reward, failed, info = observations
    x, y, omega = rescale(positions)
    return (x, y, omega), reward


# resets environment to random position and returns the initial position and velocity
def reset_environment(env):
    observations = env.reset()
    x, y, omega = rescale(observations)
    return x, y, omega


def main():
    env = gym.make('Pendulum-v0')
    env.reset()

    f = open('gym.txt', 'w')
    f.write('epoch\treward\n')

    # (x position, y position, velocity)
    state_dimensions = (X, Y, OMEGAS)

    # Initialise cumulative td_errors so that the policy starts as doing nothing in all states
    td_errors = 8 * np.ones(state_dimensions + (ACTIONS,))

    actor_critic = ac.ActorCritic(ACTIONS, state_dimensions, discount=0.95, learning_rate=0.5, td_errors=td_errors,
                                  temperature_parameter=50)

    epochs = 500
    exploration = epochs / 2  # epoch after which agent stops exploring
    display = epochs / 2  # epoch after which environment renders
    for epoch in range(epochs):
        print epoch

        # reset environment and extract initial state
        state = reset_environment(env)

        cumulative_reward = 0
        for step in range(300):

            # Get a random action if it's in the exploration phase, else follow policy.
            if epoch < exploration:
                action = random.randint(0, ACTIONS - 1)
            else:
                action = actor_critic.get_next_action(state)

            # perform action on environment
            torque = torque_from(action)
            new_state, reward = get_observations(torque, env)

            # critique the quality of the action
            actor_critic.critique(state, action, new_state, reward)

            cumulative_reward += reward
            state = new_state

            if epoch > display:
                env.render()

        f.write('{}\t{}\n'.format(epoch, cumulative_reward))
        if epoch > exploration:
            print("Following policy...")
    f.close()


if __name__ == '__main__':
    main()
