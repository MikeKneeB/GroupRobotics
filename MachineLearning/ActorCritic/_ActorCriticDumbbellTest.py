import time

from MachineLearning.Models import DumbellEnvironment as de


import ActorCritic as ac
import numpy as np

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
    env = de.Dumbell()
    for discount in [0.6, 0.8, 0.9, 0.95, 0.97, 0.99]:
        env.reset()
        filename = 'dbdis{}.txt'.format(discount)

        f = open(filename, 'w')
        env.reset()
        # (x position, y position, velocity)
        state_dimensions = (X, Y, OMEGAS)

        # If temperature_parameter is too low, you will get NaN errors
        actor_critic = ac.ActorCritic(ACTIONS, state_dimensions, discount=0.95, learning_rate=0.5, temperature_parameter=10)

        epochs = 500
        display = 500  # epoch after which environment renders
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

                if epoch > display:
                    time.sleep(0.01)
                    env.render()

            f.write('{}\t{}\n'.format(epoch, cumulative_reward))
        f.close()


if __name__ == '__main__':
    main()
