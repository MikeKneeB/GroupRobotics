import random

import gym
import numpy as np

import ActorCritic as ac

# left, down, right, up
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


# Shows the current policy. Not actually sure this works.
def show_policy(policy):
    current = policy.argmax(1)
    actions = ["<", "v", ">", "/\\"]
    out = np.array(np.zeros(16, np.string_))
    for i in range(16):
        out[i] = actions[current[i]]
    out = out.reshape((4, 4))
    out[0][0] = "S"
    out[3][3] = "G"
    out[1][1] = "H"
    out[1][3] = "H"
    out[2][3] = "H"
    out[3][0] = "H"
    print out


def main():
    f = open('grid_test.txt', 'w')
    f.write('epoch\treward\n')

    env = gym.make('FrozenLake-v0')
    state_dimensions = (16,)

    # Initialise TD errors such that initial policy is to always go down
    td_errors = np.zeros(state_dimensions + (ACTIONS,))
    td_errors[:, 1] = 5

    actor_critic = ac.ActorCritic(ACTIONS, state_dimensions, discount=0.9, learning_rate=0.5, td_errors=td_errors,
                                  temperature_parameter=2)

    epochs = 10000
    explore = epochs / 2
    for epoch in range(epochs):
        # reset environment and extract initial state
        state = reset_environment(env)

        reward = 0
        done = False
        while not done:

            # Get a random action if it's in the exploration phase, else follow policy.
            if epoch < explore:
                action = random.randint(0, 3)
            else:
                action = actor_critic.get_next_action(state)

            # perform action on environment
            new_state, reward, done = get_observations(action, env)

            # critique the quality of the action
            actor_critic.critique(state, action, new_state, reward)

            state = new_state
        f.write('{}\t{}\n'.format(epoch, reward))
        print ("Epoch: ", epoch, "\t Reward: ", reward)
    f.close()


if __name__ == '__main__':
    main()
