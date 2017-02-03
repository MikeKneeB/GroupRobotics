import random
import time
import gym
import numpy as np
from keras.models import Sequential, load_model
from keras.layers.core import Dense, Dropout, Activation, Flatten
from keras.optimizers import RMSprop, SGD

env = gym.make('Pendulum-v0')

D_ACTION_SIZE = 101
ACTION_MIN = -2
ACTION_MAX = 2

def create_actor():
    actor_model = Sequential()
    #actor_model.add(Flatten(input_shape=(1,) + env.observation_space.shape))

    actor_model.add(Dense(400, init='lecun_uniform', input_shape=(env.observation_space.shape)))
    actor_model.add(Activation('relu'))

    actor_model.add(Dense(300, init='lecun_uniform'))
    actor_model.add(Activation('relu'))

    actor_model.add(Dense(D_ACTION_SIZE, init='lecun_uniform'))
    actor_model.add(Activation('linear')) # Could change to linear.

    #Maybe try using ADAM optimiser, at any rate need to understand this code.
    a_optimizer = SGD(lr=0.1, decay=1e-6, momentum=0.9, nesterov=True)
    actor_model.compile(loss='mse', optimizer=a_optimizer)

    return actor_model

def create_critic():
    #act_inp = Input(shape=(env.action_space.shape[0],), name='act_inp')
    #obs_inp = Flatten()Input(shape=(1,)+env.observation_space.shape, name='obs_inp')

    #temp_layer = merge([act_inp, obs_inp], mode='concat')

    #temp_layer = Dense(400, init='lecun_uniform')(temp_layer)
    #temp_layer = Activation('relu')(temp_layer)

    #temp_layer = Dense(300, init='lecun_uniform')(temp_layer)
    #temp_layer = Activation('relu')(temp_layer)

    #temp_layer = Dense(1)(temp_layer)
    #temp_layer = Activation('linear')(temp_layer)

    #critic_model = Model(input=[act_inp, obs_inp], output=temp_layer)

    critic_model = Sequential()
    critic_model.add(Dense(400, init='lecun_uniform', input_shape=(env.observation_space.shape)))
    critic_model.add(Activation('relu'))

    critic_model.add(Dense(300, init='lecun_uniform'))
    critic_model.add(Activation('relu'))

    critic_model.add(Dense(1, init='lecun_uniform'))
    critic_model.add(Activation('linear'))

    c_optimizer = SGD(lr=0.1, decay=1e-6, momentum=0.9, nesterov=True)
    critic_model.compile(loss='mse', optimizer=c_optimizer)

    return critic_model


def train(epochs = 1000, run_length = 300, batch_size = 40, gamma = 0.95, epsilon = 1, min_epsilon = 0.1, buffer = 80, critic_path=None, actor_path=None):

    #actor_model = ACNetworks.ActorNet.create_actor()
    #critic_model = ACNetworks.CriticNet.create_critic()

    if actor_path == None or critic_path == None:
        actor_model = create_actor()
        critic_model = create_critic()
    else:
        actor_model = load_model(actor_path) 
        critic_model = load_model(critic_path)

    actor_replay = []
    critic_replay = []

    for epo in range(epochs+1):
        print('Game: %s' % epo)
        raw_obs = env.reset()

        scale_obs = np.array(raw_obs)
        scale_obs[0] = (scale_obs[0]+1)/2
        scale_obs[1] = (scale_obs[1]+1)/2
        scale_obs[2] = (scale_obs[2]+8)/16

        reward = 0

        reward_1 = reward[:]

        obs_1 = scale_obs[:]

        if epo == epochs - 10:
            raw_input('Enter to continue.')

        for j in range(run_length):
            if epo % 10 == 0 or epo >= epochs - 10:
                env.render()

            critic_1 = critic_model.predict(obs_1.reshape(1, env.observation_space.shape[0]))
            print('Critic 1: {}'.format(critic_1))

            if (random.random() < epsilon):
                print('Rand action')
                action = random.randint(0, 100) 
            else:
                action = actor_model.predict(obs_1.reshape(1, env.observation_space.shape[0]))
                print('Action: {}'.format(action))
                action = np.argmax(action) 

            print('Act int: {}, Act val: {} [{}]'.format(action, undiscrete_action(action), epo))
            obs_2, reward_2, d, i = env.step([undiscrete_action(action),0])
            obs_2[0] = (obs_2[0]+1)/2
            obs_2[1] = (obs_2[1]+1)/2
            obs_2[2] = (obs_2[2]+8)/16

            critic_2 = critic_model.predict(obs_2.reshape(1, env.observation_space.shape[0]))

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
                minibatch = random.sample(critic_replay, batch_size)
                x_train = []
                y_train = []
                for memory in minibatch:
                    memory_state, memory_value = memory
                    y = np.empty([1])
                    y[0] = memory_value

                    x_train.append(memory_state.reshape(env.observation_space.shape[0],)) #WHY?
                    y_train.append(y.reshape(1,))
                
                x_train = np.array(x_train)
                y_train = np.array(y_train)

                critic_model.fit(x_train, y_train, batch_size=batch_size, nb_epoch=1, verbose=0)

            while len(actor_replay) > buffer:
                actor_replay.pop(0)

            if len(actor_replay) >= buffer:
                minibatch = random.sample(actor_replay, batch_size)
                x_train = []
                y_train = []
                for memory in minibatch:
                    m_orig_state, m_action, m_value = memory
                    old_qval = actor_model.predict(m_orig_state.reshape(1, env.observation_space.shape[0]))

                    y = np.zeros(( 1, D_ACTION_SIZE ))
                    y[:] = old_qval[:]
                    y[0][m_action] = m_value
                    x_train.append(m_orig_state.reshape((env.observation_space.shape[0],)))
                    y_train.append(y.reshape((D_ACTION_SIZE,)))
                x_train = np.array(x_train)
                y_train = np.array(y_train)

                actor_model.fit(x_train, y_train, batch_size=batch_size, nb_epoch=1, verbose=0)

            obs_1 = obs_2[:]
            reward_1 = reward_2
            reward += reward_2

        if epsilon > min_epsilon:
            epsilon -= (1.0/epochs)
        print('Reward earned: {}'.format(reward))
    
    critic_model.save('critic_model.h5')
    actor_model.save('actor_model.h5')

def discrete_action(action):
    #Convert torque -2 to 2 to discrete value 0 to 100
    #D_ACTION_SIZE is define as 101 because of array length shenanigans.
    return (int)((action - ACTION_MIN)/(ACTION_MAX-ACTION_MIN)*(D_ACTION_SIZE-1))
    
def undiscrete_action(action):
    #Available action integers go from 0 to 100, need to be in range -2 to 2
    return (float)(action-50)/(25)

if __name__ == '__main__':
    train(epochs=1000)
