import numpy as np
import gym

from keras.models import Sequential
from keras.layers import Dense, Activation, Flatten
from keras.optimizers import Adam

from QNet import DeepQNetwork
import Policy
import Memory


ENV_NAME = 'Pendulum-v0'


# Start a gym environment and initialise random properties.
env = gym.make(ENV_NAME)
np.random.seed(111)
env.seed(111)

# Define parameters for network and actions
number_actions = 21            # number of discrete output actions - should be odd
gamma=0.99                    # discount factor - high for later rewards
batch_size=32                 # size of replay memory batches
train_episodes= 200           # number of training episodes probably dont need this many
test_episodes=20              # number of test episodes after training
train_steps=500               # number of steps per training episode
test_steps=300                # number of steps per test episode
replay_memory_buffer = 50000  # max size of replay buffer

# build a simple network to train, using the state shape of the environment.
model = Sequential()
model.add(Flatten(input_shape=(1,) + env.observation_space.shape))
model.add(Dense(16))
model.add(Activation('relu'))
model.add(Dense(16))
model.add(Activation('relu'))
model.add(Dense(16))
model.add(Activation('relu'))
model.add(Dense(number_actions))
model.add(Activation('linear'))    # linear output for q values

# initialise the replay memory and action selection policy
memory = Memory.ReplayMemory(replay_memory_buffer)
policy = Policy.BoltzmannQPolicy()

# create the Q network and complile it
dqn = DeepQNetwork(model=model, policy=policy,number_actions=number_actions, memory=memory, gamma=gamma, batch_size=batch_size)
dqn.compile(Adam(lr=1e-3), metrics=['mae'])

# train the network, much quicker if not visualied
dqn.train(env, episodes=train_episodes, steps=train_steps, visualize=True)

# After training save the final weights.
dqn.save_weights('dqn_{}_weights.h5f'.format(ENV_NAME), overwrite=True)

# Finally, test the trained network
dqn.test(env, episodes=test_episodes, steps=test_steps, visualize=True)