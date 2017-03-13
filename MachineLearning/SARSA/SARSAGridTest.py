import gym
import numpy as np
import SARSA as s

# left, down, right, up
ACTIONS = 4


def get_observations(action, env):
    observations = env.step(action)
    position, reward, done, info = observations
    return (position,), reward, done


# resets environment to random position and returns the initial position and velocity
def reset_environment(env):
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
    print (out)


def main():
    f = open('grid_test.txt', 'w')
    f.write('epoch\treward\n')

    env = gym.make('FrozenLake-v0')
    state_dimensions = (16,)

    # NB: if temperature_parameter is too low, you will get NaN errors
    sarsa = s.SARSA(ACTIONS, state_dimensions, discount=0.7, learning_rate=0.9, temperature_parameter=2)

    epochs = 10000
    for epoch in range(epochs):
        # reset environment and extract initial state
        state = reset_environment(env)
        action = 0
        reward = 0
        done = False

        while not done:
            new_action = sarsa.get_next_action(state)

            # perform action on environment
            new_state, reward, done = get_observations(new_action, env)

            if reward == 0:
                sarsa.update_Policy(state, action, new_state, new_action, reward)

            if reward==1 :
                sarsa.update_Policy(state, action, new_state, new_action, reward)

            state = new_state
            action = new_action
        f.write('{}\t{}\n'.format(epoch, reward))
        print("Epoch: ", epoch, "  Reward: ", reward)
    f.close()
    #show_policy(sarsa.policy)


if __name__ == '__main__':
    main()
