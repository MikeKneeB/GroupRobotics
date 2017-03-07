import random

import gym
import numpy as np

import ActorCritic as ac

# left, down, right, up
ACTIONS = 4
EPOCHS = 20000


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


def run(explore_factor, learning_rate, discount):
    filename = 'grid_{}_{}_{}_no3.txt'.format(explore_factor, learning_rate, discount)
    f = open(filename, 'w')

    # initialise the policy by setting the cumulative TD error
    state_dimensions = (16,)
    td_errors = np.zeros(state_dimensions + (ACTIONS,))
    td_errors[:, 1] = 5

    actor_critic = ac.ActorCritic(ACTIONS, state_dimensions, discount, learning_rate, td_errors)

    explore = EPOCHS * explore_factor
    f.write('epoch\treward\n')
    for epoch in range(EPOCHS):
        # reset environment and extract initial state
        state = reset_enviroment(env)
        done = False
        reward = 0
        while not done:
            if epoch > explore:
                action = actor_critic.get_next_action(state)
            else:
                action = random.randint(0, 3)

            # perform action on environment
            new_state, reward, done = get_observations(action, env)

            # critique the quality of the action
            actor_critic.critique(state, action, new_state, reward)
            state = new_state

            # print display, ",", epochs
            # env.render()
            # print "\n"

        f.write('{}\t{}\n'.format(epoch, reward))
        # if epoch > 39000:
        #     time.sleep(1)
        #     print "########"
        #     show_policy(actor.actor.policy)
    f.close()


def main():
    explore_factor = 0.5
    i = 0
    for learning_rate in np.arange(0.05, 1, 0.05):
        for discount in np.arange(0.25, 1, 0.05):
            i += 1
            run(explore_factor, learning_rate, discount)
            print('Number {}'.format(i))


if __name__ == '__main__':
    env = gym.make('FrozenLake-v0')
    main()
