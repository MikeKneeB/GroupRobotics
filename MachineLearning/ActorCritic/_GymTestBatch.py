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
    for learning_rate in [0.25, 0.5, 0.75, 1]:
        for temperature_parameter in [1, 10, 100, 1000]:

            env.reset()
            filename = 'pendl{}t{}.txt'.format(learning_rate, temperature_parameter)

            f = open(filename, 'w')
            f.write('epoch\treward\n')

            # (x position, y position, velocity)
            state_dimensions = (X, Y, OMEGAS)

            # If temperature_parameter is too low, you will get NaN errors
            actor_critic = ac.ActorCritic(ACTIONS, state_dimensions, 0.9, learning_rate=learning_rate,
                                          temperature_parameter=temperature_parameter)

            epochs = 1000
            for epoch in range(epochs):
                print(epoch)

                # reset environment and extract initial state
                state = reset_environment(env)

                cumulative_reward = 0
                for step in range(300):

                    action = actor_critic.get_next_action(state)

                    # perform action on environment
                    torque = torque_from(action)
                    new_state, reward = get_observations(torque, env)

                    # critique the quality of the action
                    actor_critic.critique(state, action, new_state, reward)

                    cumulative_reward += reward
                    state = new_state

                f.write('{}\t{}\n'.format(epoch, cumulative_reward))
            f.close()


if __name__ == '__main__':
    main()
