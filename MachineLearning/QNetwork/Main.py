from keras.models import Sequential
from keras.layers import Dense, Activation, Flatten
from keras.optimizers import Adam

from MachineLearning.Models.DumbellEnvironmentNonRepeat import Dumbell
from MachineLearning.QNetwork.QNet import DeepQNetwork
import MachineLearning.QNetwork.Policy as Policy
import MachineLearning.QNetwork.Memory as Memory


ENV_NAME = 'Pendulum-v0'   # Only one that works, need different policies for others
# ENV_NAME = 'CartPole-v0'
# ENV_NAME = 'MountainCar-v0'


# Start a gym environment and initialise random properties.
# env = gym.make(ENV_NAME)
env = Dumbell(1.8, 0.5, 0.9)
seed = 0                    # random seed, useful for checking changes set to 0 and comment bellow to remove effect
# np.random.seed(seed)      # comment these to remove seed
# env.seed(seed)

# Define parameters for network and actions
# number of discrete output actions
number_actions = 3              # total number of discreet actions
min_action = -5                 # minimum discreet action, max taken to be -min.
initial_tau = 100.              # exploration, higher the more random the action selection
tau_decay_rate = 1.2            # decay of tau, must be greater than 1. When = 1 constant tau.
gamma = 0.99                    # discount factor
batch_size = 64                 # size of replay memory batches
train_episodes = 100            # number of training episodes
test_episodes = 100             # number of test episodes after training
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
memory = Memory.ReplayMemory(replay_memory_buffer, seed)
policy = Policy.BoltzmannQPolicy()
policy.set_tau(initial_tau)     # initialise policy tau

# create the Q network and compile it
dqn = DeepQNetwork(model, policy, number_actions, memory, gamma, batch_size,
                   env, min_action, initial_tau, tau_decay_rate)
dqn.compile(Adam(lr=1e-3))

# train the network, much quicker if not visualied
dqn.train(episodes=train_episodes, steps=train_steps, visualize=True)

# After training save the final weights.
dqn.save_weights('dqn_{}_weights.h5f'.format(ENV_NAME), overwrite=True)

# Finally, test the trained network
dqn.test(episodes=test_episodes, steps=test_steps, visualize=False)
