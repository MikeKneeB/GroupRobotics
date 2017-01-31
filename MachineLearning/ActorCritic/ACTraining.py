import random
import time
import gym
import numpy as np

import ACNetworks as ACN

def train(epochs = 1000, run_length = 300, batch_size = 40, gamma = 0.95, epsilon = 1, min_epsilon = 0.1, buffer = 80):

    env = gym.make('Pendulum-v0')

    #actor_model = ACNetworks.ActorNet.create_actor()
    #critic_model = ACNetworks.CriticNet.create_critic()

    actor_model = ACN.create_actor()
    critic_model = ACN.create_critic()

    actor_replay = []
    critic_replay = []

    for epo in range(epochs):
        print('Game: %s' % epo)
        raw_obs = env.reset()

        scale_obs = np.array(raw_obs)
        scale_obs[0] = (scale_obs[0]+1)/2
        scale_obs[1] = (scale_obs[1]+1)/2
        scale_obs[2] = (scale_obs[2]+8)/16

        reward = 0

        obs_1 = scale_obs[:]

        for j in range(run_length):
            if epo % 10 == 0:
                env.render()

            reward_1 = reward

            critic_1 = critic_model.predict(obs_1.reshape(1, ACN.state_dimension))

            if (random.random() < epsilon):
               action = env.action_space.sample() 
            else:
                action = actor_model.predict(obs_1.reshape(1, ACN.state_dimension))
                action = (action * 4) - 2

            obs_2, reward_2, d, i = env.step(action)
            obs_2[0] = (obs_2[0]+1)/2
            obs_2[1] = (obs_2[1]+1)/2
            obs_2[2] = (obs_2[2]+8)/16

            critic_2 = critic_model.predict(obs_2.reshape(1, ACN.state_dimension))

            if j == run_length - 1:
                target = reward_1 + (gamma * reward_2)
            else:
                target = reward_1 + (gamma * critic_2)

            critic_update_val = max((critic_1 * gamma), target)

            critic_replay.append([obs_1, critic_update_val])

            if j == run_length - 1:
                critic_replay.append([obs_2, reward_2])

            actor_delta = critic_2 - critic_1
            actor_replay.append([obs_1, action, actor_delta])

            while len(critic_replay) > buffer:
                critic_replay.pop(0)

            if len(critic_replay) >= buffer:
                #print('Critic Buffer.')
                minibatch = random.sample(critic_replay, batch_size)
                x_train = []
                y_train = []
                for memory in minibatch:
                    memory_state, memory_value = memory
                    y = np.empty([1])
                    y[0] = memory_value

                    x_train.append(memory_state.reshape(ACN.state_dimension,)) #WHY?
                    y_train.append(y.reshape(1,))
                
                x_train = np.array(x_train)
                y_train = np.array(y_train)

                critic_model.fit(x_train, y_train, batch_size=batch_size, nb_epoch=1, verbose=0)

            while len(actor_replay) > buffer:
                actor_replay.pop(0)

            if len(actor_replay) >= buffer:
                #print('Actor Buffer.')
                minibatch = random.sample(actor_replay, batch_size)
                x_train = []
                y_train = []
                for memory in minibatch:
                    memory_state, memory_action, memory_value = memory
                    y = np.empty([1])
                    y[0] = memory_value

                    x_train.append(memory_state.reshape(ACN.state_dimension,))
                    y_train.append(y.reshape(1,))

                x_train = np.array(x_train)
                y_train = np.array(y_train)

                actor_model.fit(x_train, y_train, batch_size=batch_size, nb_epoch=1, verbose=0)

            obs_1 = obs_2[:]
            reward_1 = reward_2

        if epsilon > min_epsilon:
            epsilon -= (1/epochs)

if __name__ == '__main__':
    train()
