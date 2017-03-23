"""
Author: Jay Morris
Date: 18/03/17

Test of SARSA on frozen lake environment
"""
import gym
import numpy as np
import SARSA as s

# left, down, right, up
ACTIONS = 4


#gets observables for the environment for an inputted action
def get_observations(action, env):
    observations = env.step(action)
    position, reward, done, info = observations
    return (position,), reward, done


# resets environment to random position and returns the initial position and velocity
def reset_environment(env):
    observations = env.reset()
    return observations,


#prints out the given policy in a grid
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
    print(out)


#runs the sarsa algorithm on the environment
def main():
    #opens a text file to output to
    f = open('grid_test.txt', 'w')
    f.write('epoch\treward\n')

    #intitialises the environment
    env = gym.make('FrozenLake-v0')
    state_dimensions = (16,)

    # NB: if temperature_parameter is too low, you will get NaN errors
    #initialises a sarsa object
    learning_rate = 0.5
    sarsa = s.SARSA(ACTIONS, state_dimensions, discount=0.9, learning_rate = learning_rate, temperature_parameter=2)

    epochs = 10000
    for epoch in range(epochs):
        # reset environment and extract initial state
        state = reset_environment(env)
        action = sarsa.get_next_action(state)
        reward = 0
        done = False

        while not done:
            # perform action on environment
            new_state, reward, done = get_observations(action, env)

            #get the next action to be performed
            new_action = sarsa.get_next_action(new_state)

            #if clause for ease of testing. Allows exceptions to be made only when Q-Values are updated notably
            if reward == 0:
                #updates the policy based on inputs
                sarsa.update_Policy(state, action, new_state, new_action, reward)

            if reward==1 :
                sarsa.update_Policy(state, action, new_state, new_action, reward)

            #updates the state and action to the next one
            state = new_state
            action = new_action

        #potential fix by slowly adjusting learning rate, can be left commented out
        sarsa.learningRate = learning_rate - (0.5/10000)
        f.write('{}\t{}\n'.format(epoch, reward))
        print("Epoch: ", epoch, "  Reward: ", reward)
    f.close()
    #show_policy(sarsa.policy)


#runs the main method when this file is run
if __name__ == '__main__':
    main()
