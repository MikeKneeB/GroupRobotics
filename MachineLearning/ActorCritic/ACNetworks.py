from keras.models import Sequential
from keras.layers.core import Dense, Dropout, Activation
from keras.optimizers import RMSprop, SGD
import gym
    
state_dimension = 3
action_dimension = 1

def create_actor():
    actor_model = Sequential()
    actor_model.add(Dense(400, init='lecun_uniform', input_shape=(state_dimension,)))
    actor_model.add(Activation('relu'))

    actor_model.add(Dense(300, init='lecun_uniform'))
    actor_model.add(Activation('relu'))

    actor_model.add(Dense(action_dimension, init='lecun_uniform'))
    actor_model.add(Activation('tanh'))

    #Maybe try using ADAM optimiser, at any rate need to understand this code.
    a_optimizer = SGD(lr=0.1, decay=1e-6, momentum=0.9, nesterov=True)
    actor_model.compile(loss='mse', optimizer=a_optimizer)

    return actor_model


def create_critic():
    critic_model = Sequential()
    critic_model.add(Dense(400, init='lecun_uniform', input_shape=(state_dimension,)))
    critic_model.add(Activation('relu'))

    critic_model.add(Dense(300, init='lecun_uniform'))
    critic_model.add(Activation('relu'))

    critic_model.add(Dense(1, init='lecun_uniform'))
    critic_model.add(Activation('linear'))

    c_optimizer = SGD(lr=0.1, decay=1e-6, momentum=0.9, nesterov=True)
    critic_model.compile(loss='mse', optimizer=c_optimizer)

    return critic_model

