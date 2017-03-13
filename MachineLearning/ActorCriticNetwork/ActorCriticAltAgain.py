# https://gym.openai.com/evaluations/eval_n7JgacQRiK3MMrWFnaz6g

import tensorflow as tf
import tflearn
import numpy as np
import random
import datetime

from MachineLearning.Models import DumbellEnvironmentNonRepeatContinuous as denv
from MachineLearning.ActorCriticNetwork import ActorCriticTrain as ACT
import actor
import critic

def dumbellObsComp(obs):
    #obs = [obs[0][0][0], obs[1][0][0]]
    #obs = np.array([np.sin(obs[0]), np.cos(obs[0]), obs[1]])
    return obs.reshape(state_dim, 1)

def dumbellRewComp(reward):
    return reward.reshape(1)

if __name__ == '__main__':
    with tf.Session() as sess:
        # Make our environment.
        env = denv.Dumbell()

        # Get environment params, for building networks etc.
        state_dim = 4
        action_dim = 1
        max_action = 5

        print('{} {} {}'.format(state_dim, action_dim, max_action))

        # Build our actor and critic agents.
        actor_model = actor.ActorNetwork(sess, state_dim, action_dim, max_action, 0.0001, 0.001)
        critic_model = critic.CriticNetwork(sess, state_dim, action_dim, max_action, 0.001, 0.001, actor_model.get_num_trainable_vars())

        # Train.
        ACT.train(sess, actor_model, critic_model, env, state_dim, action_dim, max_action, epochs=200, run_length=500, decay=0.98,
        render=False, buffer=1000, envname='dumbell_nr', obsComp=dumbellObsComp, rewComp=dumbellRewComp)
