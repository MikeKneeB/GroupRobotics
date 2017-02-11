# https://gym.openai.com/evaluations/eval_n7JgacQRiK3MMrWFnaz6g

import tensorflow as tf
import tflearn
import numpy as np
import gym
import random
import datetime

import actor
import critic

"""
train trains the neural network objects using the tensorflow session sess.
Can be run with varying parameters, though recommended defaults will be fine
in many cases. Environments can be any kind of continuous openai env.

sess: tensorflow session to run on.
actor_model: actor network object.
critic_model: critic network object.
env: openai environment.
epochs: number of training runs to run.
run_length: number of actions available to the agent in one run.
batch_size: memory batch for online training.
gamma: learning rate (set high).
epsilon: noise range.
min_epsilon: smallest epsilon to use (reached at final run of training).
buffer: memory buffer size (number of stored memories).
filepath: optional filepath to save data too, in most cases not needed.
render: show environment state during training.
"""
def train(sess, actor_model, critic_model, env, epochs = 1000, run_length = 300, batch_size = 40, gamma = 0.95, epsilon = 1, min_epsilon = 0.01, buffer = 1000, filepath=None, render=False):

    now = datetime.datetime.now()

    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]

    max_action = env.action_space.high

    if filepath is None: 
        filename = '{}:{}_{}-{}-{}_e{}-l{}_m{}-b{}_ep{}'.format(now.hour, now.minute, now.day, now.month, now.year, epochs, run_length, buffer, batch_size, epsilon)
    else:
        filename = filepath

    with open(filename, 'w') as f:

        f.write('{}\t{}\n'.format('ep','re'))

        sess.run(tf.global_variables_initializer())

        actor_model.update_target_network()
        critic_model.update_target_network()

        replay = []

        for epo in range(epochs+1):
            print('Game: %s' % epo)

            obs_1 = env.reset()

            reward_total = 0
            reward = 0

            for j in range(run_length):
                if render:
                    env.render()

                print('Obs: {}'.format(obs_1.reshape(1, state_dim)))

                noise_r = epsilon/2.
                action = actor_model.predict(obs_1.reshape(1, state_dim)) + random.uniform(-noise_r, noise_r)

                print('Act val: {} [{}, {}]'.format(action, epo, j))

                obs_2, reward, d, i = env.step(action)

                replay.append([np.reshape(obs_1, state_dim), np.reshape(action, action_dim), reward, np.reshape(obs_2, state_dim)])

                while len(replay) > buffer:
                    replay.pop(0)

                if len(replay) >= buffer:
                    mem = random.sample(replay, batch_size)
                    o_batch = column(mem, 0)
                    a_batch = column(mem, 1)
                    r_batch = column(mem, 2)
                    o2_batch = column(mem, 3)

                    q_batch = critic_model.predict_target(o2_batch, actor_model.predict_target(o2_batch))

                    y_train = []

                    for k in range(batch_size):
                        y_train.append(r_batch[k] + gamma*q_batch[k])

                    predict_q, _ = critic_model.train(o_batch, a_batch, y_train)

                    a_out = actor_model.predict(o_batch)
                    a_gradient = critic_model.action_gradients(o_batch, a_out)
                    actor_model.train(o_batch, a_gradient[0])

                    actor_model.update_target_network()
                    critic_model.update_target_network()

                obs_1 = obs_2[:]

                if epsilon > min_epsilon:
                    epsilon -= (1.0/(epochs*run_length))
                    if epsilon < min_epsilon:
                        epsilon = min_epsilon

                reward_total += reward

                if d:
                    continue

            print('Reward earned: {}'.format(reward_total))
            f.write('{}\t{}\n'.format(epo, reward_total[0]))

        f.close()

"""
Test tests a trained agent (both nets) on an environment. This does not produce  
data and always renders.

sess: tensorflow learn session.
actor_model: actor object (should already be trained).
critic_model: critic object (should also already be trained).
env: openai environment.
epochs: number of test runs.
run_length: number of actions available to the agent in one run.
"""
def test(sess, actor_model, critic_model, env, epochs = 1000, run_length = 300):
    for epo in range(epochs+1):
        print('Game: %s' % epo)

        obs_1 = env.reset()

        reward_total = 0
        reward = 0

        for j in range(run_length):
            env.render()

            print('Obs: {}'.format(obs_1.reshape(1, state_dim)))

            action = actor_model.predict(obs_1.reshape(1, state_dim))

            print('Act val: {} [{}, {}]'.format(action, epo, j))

            obs_2, reward, d, i = env.step(action)

            obs_1 = obs_2[:]

            reward_total += reward

            if d:
                continue

        print('Reward earned: {}'.format(reward_total))
 
    return 0

"""
Returns a column of a list of lists as a single array.

matrix: list of lists to get column.
i: column index.
"""
def column(matrix, i):
    return [row[i] for row in matrix]

if __name__ == '__main__':
    with tf.Session() as sess:
        env = gym.make('Pendulum-v0')

        state_dim = env.observation_space.shape[0]
        action_dim = env.action_space.shape[0]
        max_action = env.action_space.high

        actor_model = actor.ActorNetwork(sess, state_dim, action_dim, max_action, 0.0001, 0.001) 
        critic_model = critic.CriticNetwork(sess, state_dim, action_dim, max_action, 0.001, 0.001, actor_model.get_num_trainable_vars()) 

        train(sess, actor_model, critic_model, env, epochs=500, run_length=300, render=False)

        raw_input('Training complete, press enter to continue to test.')

        test(sess, actor_model, critic_model, env, epochs=50, run_length=300)
