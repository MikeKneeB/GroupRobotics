import random

import time
import numpy as np
import gym

import ActorCritic as ac

# left, down, right, up
ACTIONS = 4


def get_observations(action, env):
    observations = env.step(action)
    position, reward, done, info = observations
    return (position,), reward, done


# resets enviroment to random position and returns the initial position and velocity
def reset_enviroment(env):
    observations = env.reset()
    return observations,


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
    env = gym.make('FrozenLake-v0')
    gym.spec
    f = open('shawac.txt', 'w')

    # (positions, velocities)
    state_dimensions = (16,)

    discount = 0.7
    time_horizon = 10

    actor = ac.ActorCritic(ACTIONS, time_horizon, state_dimensions, discount, 10)

    f.write('epoch\treward\n')
    epochs = 20000
    explore = epochs / 2
    for epoch in range(epochs):
        # reset environment and extract initial state
        state = reset_enviroment(env)
        done = False
        reward = 0
        while not done:
            if epoch > explore:
                action = actor.get_next_action(state)
            else:
                action = random.randint(0, 3)

            # perform action on environment
            new_state, reward, done = get_observations(action, env)

            # critique the quality of the action
            actor.critique(state, action, new_state, reward)
            state = new_state

            # print display, ",", epochs
            env.render()
            print "\n"

        f.write('{}\t{}\n'.format(epoch, reward))
        print"Epoch: ", epoch, "\t Reward: ", reward, "\t Exploring: ", epoch < explore
        if epoch > 1000:
            time.sleep(1)
            print "########"
            #show_policy(actor.actor.policy)
    f.close()


if __name__ == '__main__':
    main()
