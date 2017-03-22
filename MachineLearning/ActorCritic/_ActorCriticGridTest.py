"""
Author: Harry Shaw
Date: 18/03/17

Test of actor-critic on frozen lake environment
"""

import gym

import ActorCritic as ac

ACTIONS = 4


# Get observations of environment after performing an action.
def get_observations(action, env):
    observations = env.step(action)
    position, reward, done, info = observations
    return (position,), reward, done


# resets environment to random position and returns the initial position and velocity
def reset_environment(env):
    observations = env.reset()
    return observations,


def main():
    f = open('grid_test.txt', 'w')
    f.write('epoch\treward\n')

    # outputs epoch and cumulative reward gained during an epoch
    env = gym.make('FrozenLake-v0')
    state_dimensions = (16,)

    # NB: if temperature_parameter is too low, you will get NaN errors
    actor_critic = ac.ActorCritic(ACTIONS, state_dimensions, discount=0.9, learning_rate=0.5, temperature_parameter=2)

    epochs = 10000
    for epoch in range(epochs):
        # reset environment and extract initial state
        state = reset_environment(env)

        reward = 0
        done = False
        while not done:
            action = actor_critic.get_next_action(state)

            # perform action on environment
            new_state, reward, done = get_observations(action, env)

            # critique the quality of the action
            actor_critic.critique(state, action, new_state, reward)

            state = new_state
        f.write('{}\t{}\n'.format(epoch, reward))
        print("Epoch: ", epoch, "  Reward: ", reward)
    f.close()


if __name__ == '__main__':
    main()
