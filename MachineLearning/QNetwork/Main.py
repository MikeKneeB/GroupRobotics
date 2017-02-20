import numpy as np
import gym
import Bumbell

from keras.models import Sequential
from keras.layers import Dense, Activation, Flatten
from keras.optimizers import Adam

from QNetTest import DeepQNetwork
import Policy
import Memory


ENV_NAME = 'Pendulum-v0'   # only one that works, need different policies for others
#ENV_NAME = 'CartPole-v0'
#ENV_NAME = 'MountainCar-v0'


# Start a gym environment and initialise random properties.
env = gym.make(ENV_NAME)
seed=111
np.random.seed(seed)
env.seed(seed)

# Define parameters for network and actions
# number of discrete output actions
number_actions = 5
gamma = 0.99                    # discount factor
batch_size = 64                 # size of replay memory batches
train_episodes = 100            # number of training episodes
test_episodes = 100              # number of test episodes after training
train_steps = 500               # number of steps per training episode
test_steps = 200                # number of steps per test episode
replay_memory_buffer = 100000   # max size of replay buffer

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
memory = Memory.ReplayMemory(replay_memory_buffer,random_seed=seed)
policy = Policy.BoltzmannQPolicy()

# create the Q network and compile it
dqn = DeepQNetwork(model=model, policy=policy,number_actions=number_actions, memory=memory, gamma=gamma, batch_size=batch_size , environment=env)
dqn.compile(Adam(lr=1e-3), metrics=['mae'])

# train the network, much quicker if not visualied
dqn.train(episodes=train_episodes, steps=train_steps, visualize=False)

# After training save the final weights.
dqn.save_weights('dqn_{}_weights.h5f'.format(ENV_NAME), overwrite=True)

# Finally, test the trained network
dqn.test(episodes=test_episodes, steps=test_steps, visualize=False)