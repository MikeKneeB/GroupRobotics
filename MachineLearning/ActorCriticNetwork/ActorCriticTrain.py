# https://gym.openai.com/evaluations/eval_n7JgacQRiK3MMrWFnaz6g

import tensorflow as tf
import tflearn
import numpy as np
import gym
import random
import datetime
import time

import subprocess

import actor
import critic

"""
train trains the neural network objects using the tensorflow session sess.
Can be run with varying parameters, though recommended defaults will be fine
in many cases. Environments can be any kind of continuous env with, at a
minimum, a reset and step function.

sess: tensorflow session to run on.
actor_model: actor network object.
critic_model: critic network object.
env: openai environment.
state_dim: dimension of the state space of env.
action_dim: dimensions of the action space of env.
epochs: number of training runs to run.
run_length: number of actions available to the agent in one run.
batch_size: memory batch for online training.
gamma: learning rate (set high).
epsilon: noise range.
min_epsilon: smallest epsilon to use (reached at final run of training).
buffer: memory buffer size (number of stored memories).
envname: environment name, for creating filepath.
render: show environment state during training. Must be False if env has no
    render method.
obsComp: optional observation comprehension function.
rewComp: optional reward comprehension function.
"""
def train(sess, actor_model, critic_model, env, state_dim, action_dim, max_action,
          epochs = 1000, run_length = 300,
          gamma = 0.95, epsilon = None, min_epsilon = 0.01, decay = 0.95,
          batch_size = 40, buffer = 1000,
          envname=None, render=False, obsComp=None, rewComp=None):
    # For file naming purposes.
    now = datetime.datetime.now()

    # Build filename.
    if envname is None:
        envname = 'NOENV'

    filepath = '{}:{}_{}-{}-{}_{}.dat'.format(now.hour, now.minute, now.day, now.month, now.year, envname)

    #if epsilon is None and type(max_action) is list:
    try:
        epsilon = 2*max_action[0]
    except TypeError:
        epsilon = 2*max_action
    #elif epsilon is None:
    #    epsilon = 2*max_action

    with open(filepath, 'w') as f:

        f.write('# Epochs: {} | Run Length: {} | Memory Size: {} | Batch Size: {} | Initial Epsilon: {} | Decay: {}\n'.format(
        epochs, run_length, buffer, batch_size, epsilon, decay))

        f.write('{:10s}{:15s}{:15s}{:s}\n'.format('ep','re','rol_re','epsilon'))

        # Initialise global variables for tensorflow session.
        sess.run(tf.global_variables_initializer())

        # Initialise target networks.
        actor_model.update_target_network()
        critic_model.update_target_network()

        # Replay is a simple list of lists, start of as an empty list.
        replay = []
        running_reward = []

        for epo in range(epochs):
            print('Game: %s' % epo)

            # Reset env and get initial observation.
            obs_1 = env.reset()
            if obsComp is not None:
                obs_1 = obsComp(obs_1)

            # reward_total for output.
            reward_total = 0

            # UNCOMMENT FOR GIFS - if you have byzanz installed.
            # if epo == 10 or epo == 50 or epo == 100 or epo == 150 or epo == 200 or epo == 250 or epo == 300:
            #     subprocess.Popen(['byzanz-record', '-x', '0', '-y', '50', '-w', '500', '-h', '500', '-d', '10', 'run_{}.gif'.format(epo)])

            for j in range(run_length):
                # Render environment if possible.
                if render:
                    env.render()

                print('Obs: {}'.format(obs_1.reshape(1, state_dim)))

                est_time_1 = time.time()

                # Calculate noise amount, set to half epsilon.
                noise_r = epsilon/2.
                # Calculate noisy action.
                action = actor_model.predict(obs_1.reshape(1, state_dim))# + random.uniform(-noise_r, noise_r)
                if action + noise_r > max_action:
                    diff = max_action - action
                    #assert diff >= 0
                    action = action + random.uniform(diff - epsilon, diff)
                elif action - noise_r < -max_action:
                    diff = -max_action - action
                    #assert diff <= 0
                    action = action + random.uniform(diff, epsilon + diff)
                else:
                    action = action + random.uniform(-noise_r, noise_r)

                print('Act val: {} [{}, {}, {}]'.format(action, epo, j, epsilon))

                # Perform action.
                obs_2, reward, d, i = env.step(action)
                if obsComp is not None:
                    obs_2 = obsComp(obs_2)
                if rewComp is not None:
                    reward = rewComp(reward)

                print(obs_2)
                print(reward)

                # Add what happened to the memory.
                replay.append([np.reshape(obs_1, state_dim), np.reshape(action, action_dim), reward, np.reshape(obs_2, state_dim)])

                # If more items in memory than necessary, remove some.
                while len(replay) > buffer:
                    replay.pop(0)

                # If we have enough memory items in replay, do some learning.
                if len(replay) >= batch_size: #buffer:
                    # Get sample from memory.
                    mem = random.sample(replay, batch_size)
                    # Batch observations.
                    o_batch = column(mem, 0)
                    # Batch actions.
                    a_batch = column(mem, 1)
                    # Batch rewards.
                    r_batch = column(mem, 2)
                    # Batch observations after taking action in state.
                    o2_batch = column(mem, 3)

                    # What are the q vals for this batch?
                    q_batch = critic_model.predict_target(o2_batch, actor_model.predict_target(o2_batch))

                    # q_val update list, using learning factor and reward earned.
                    y_train = []

                    for k in range(batch_size):
                        # Update vals.
                        y_train.append(r_batch[k] + gamma*q_batch[k])

                    # Train the critic network, using update vals and
                    # obs/actions.
                    predict_q, _ = critic_model.train(o_batch, a_batch, y_train)

                    # Predict actions actor would take.
                    a_out = actor_model.predict(o_batch)
                    # Get action gradient for observations.
                    a_gradient = critic_model.action_gradients(o_batch, a_out)
                    # Train on action gradients.
                    actor_model.train(o_batch, a_gradient[0])

                    # Update target nets.
                    actor_model.update_target_network()
                    critic_model.update_target_network()

                # Move to the next state!
                obs_1 = obs_2[:]

                # For output info.
                reward_total += reward

                # In the invpen case this never happens, but left in for other
                # environments.
                if d:
                    continue

            print('Reward earned: {}'.format(reward_total))

            if len(running_reward) > 20:
                del running_reward[0]
                print(len(running_reward))
            running_reward.append(reward_total[0])
            f.write('{:<10d}{:<15f}{:<15f}{:<15f}\n'.format(epo, reward_total[0], sum(running_reward)/len(running_reward), epsilon))

            # Update epsilon, noise factor decreases each time.
            if epsilon > min_epsilon:
                #epsilon -= (1.0/epochs) # OLD LINEAR VERSION
                epsilon = decay * epsilon # SHINY NEW EXPONENTIAL VERSION
                # Ensure epsilon does not go below minimum.
                if epsilon < min_epsilon:
                    epsilon = min_epsilon


        f.close()

"""
Test tests a trained agent (both nets) on an environment. This does not produce
data and always renders. Cannot be run on environments withouth a render
method.

sess: tensorflow learn session.
actor_model: actor object (should already be trained).
critic_model: critic object (should also already be trained).
env: openai environment.
state_dim: dimension of state space of env.
action_dim: dimensions of action space of env.
epochs: number of test runs.
run_length: number of actions available to the agent in one run.
obsComp: optional observation comprehension function.
rewComp: optional reward comprehension function.
"""
def test(sess, actor_model, critic_model, env, state_dim, action_dim,
        epochs = 1000, run_length = 300,
        obsComp=None, rewComp=None, filename='output'):

    obs_store = []
    action_store = []

    for epo in range(epochs+1):
        print('Game: %s' % epo)

        # Get initial epoch observations.
        obs_1 = env.reset()
        if obsComp is not None:
            obs_1 = obsComp(obs_1)

        for j in range(run_length):
            # Always draw in test mode.
            env.render()

            print('Obs: {}'.format(obs_1.reshape(1, state_dim)))

            # Get action, do not add noise.
            action = actor_model.predict(obs_1.reshape(1, state_dim))

            if epo == 0:
                obs_store.append([obs_1[0].item(), obs_1[1].item(), obs_1[2].item()])
                action_store.append(action[0][0])
            # else:
            #     obs_store[j] = [(x + y) for x, y in zip(obs_store[j], [obs_1[0].item(), obs_1[1].item(), obs_1[2].item()])]
            #     action_store[j] += action[0][0]

            print('Act val: {} [{}, {}]'.format(action, epo, j))

            # Step and observe.
            obs_2, reward, d, i = env.step(action)
            if obsComp is not None:
                obs_2 = obsComp(obs_2)
            if rewComp is not None:
                reward = rewComp(reward)

            # Move to our new state.
            obs_1 = obs_2[:]

            if d:
                continue

    with open(filename, 'w') as f:
        for i, item in enumerate(obs_store):
            f.write('{:<10d}{:<15f}{:<15f}{:<15f}{:<15f}\n'.format(i, item[0], item[1], item[2], action_store[i]))

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
        # Make our environment.
        env = gym.make('Pendulum-v0')
        env.seed(int(time.time()))

        # Get environment params, for building networks etc.
        state_dim = env.observation_space.shape[0]
        action_dim = env.action_space.shape[0]
        max_action = env.action_space.high

        print('{} {} {}'.format(state_dim, action_dim, max_action))

        # Build our actor and critic agents.
        actor_model = actor.ActorNetwork(sess, state_dim, action_dim, max_action, 0.0001, 0.001)
        critic_model = critic.CriticNetwork(sess, state_dim, action_dim, max_action, 0.001, 0.001, actor_model.get_num_trainable_vars())

        # Train.
        train(sess, actor_model, critic_model, env, state_dim, action_dim,
        max_action, epochs=1, run_length=200, render=True, envname='pendulum', decay=0.98)

        raw_input('Training complete, press enter to continue to test.')

        # Test.
        test(sess, actor_model, critic_model, env, state_dim, action_dim, epochs=10, run_length=300)
