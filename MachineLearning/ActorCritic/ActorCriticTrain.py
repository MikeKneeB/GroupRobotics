# https://gym.openai.com/evaluations/eval_n7JgacQRiK3MMrWFnaz6g

import tensorflow as tf
import tflearn
import numpy as np
import gym
import random
import datetime

import actor
import critic

def train(sess, actor_model, critic_model, env, epochs = 1000, run_length = 300, batch_size = 40, gamma = 0.95, epsilon = 1, min_epsilon = 0.01, buffer = 1000, filepath=None, render=False):

    now = datetime.datetime.now()
    #actor_model = ACNetworks.ActorNet.create_actor()
    #critic_model = ACNetworks.CriticNet.create_critic()

    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]

    max_action = env.action_space.high

    if filepath is not None: 
        f = open(filepath, 'w')
    else:
        f = open('{}:{}_{}-{}-{}_e{}-l{}_m{}-b{}'.format(now.hour, now.minute, now.day, now.month, now.year, epochs, run_length, buffer, batch_size))
    f.write('{}\t{}\n'.format('ep','re'))

    #if actor_path == None and critic_path == None:
    #    actor_model = create_actor()
    #    critic_model = create_critic()
    #elif actor_path == None or critic_path == None:
    #    print('If you supply one network, you must supply both.')
    #    sys.exit(1)
    #else:
    #actor_model = actor.ActorNetwork(sess, state_dim, action_dim, max_action, 0.0001, 0.001) 
    #critic_model = critic.CriticNetwork(sess, state_dim, action_dim, max_action, 0.001, 0.001, actor_model.get_num_trainable_vars()) 

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

            #if (random.random() < epsilon):
            #    print('Rand action')
            #    action = [random.uniform(-max_action, max_action)] 
            #else:
            #    action = actor_model.predict(obs_1.reshape(1, state_dim)) 
            #    print('Action: {}'.format(action))

            noise_r = epsilon/2.
            action = actor_model.predict(obs_1.reshape(1, state_dim)) + random.uniform(-noise_r, noise_r)

            print('Act val: {} [{}, {}]'.format(action, epo, j))

            obs_2, reward, d, i = env.step(action)

            replay.append([np.reshape(obs_1, state_dim), np.reshape(action, action_dim), reward, np.reshape(obs_2, state_dim)])

            while len(replay) > buffer:
                replay.pop(0)

            if len(replay) >= buffer:
                #o_batch, a_batch, r_batch, o2_batch = random.sample(replay, batch_size)
                
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

            reward_total += reward

            if d:
                continue

        print('Reward earned: {}'.format(reward_total))
        f.write('{}\t{}\n'.format(epo, reward_total[0]))

    f.close()

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

        train(sess, actor_model, critic_model, env, epochs=200, run_length=300, render=True)

        raw_input('Training complete, press enter to continue to test.')

        test(sess, actor_model, critic_model, env, epochs=50, run_length=300)
