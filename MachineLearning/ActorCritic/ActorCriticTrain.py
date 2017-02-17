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
def train(sess, actor_model, critic_model, env, state_dim, action_dim, max_action, epochs = 1000, run_length = 300, batch_size = 40, gamma = 0.95, epsilon = 1, min_epsilon = 0.01, buffer = 1000, filepath=None, render=False):

    # For file naming purposes.
    now = datetime.datetime.now()

    # Get the dimensions of this environment (no longer)
    # state_dim = env.observation_space.shape[0]
    # action_dim = env.action_space.shape[0]
    # max_action = env.action_space.high

    # Build filename if required.
    if filepath is None:
        filepath = '{}:{}_{}-{}-{}'.format(now.hour, now.minute, now.day, now.month, now.year)

    with open(filepath, 'w') as f:

        f.write('# Epochs: {} | Run Length: {} | Memory Size: {} | Batch Size: {} | Initial Epsilon: {}\n'.format(epochs, run_length, buffer, batch_size, epsilon))

        f.write('{:10s}{:s}\n'.format('ep','re'))

        # Initialise global variables for tensorflow session.
        sess.run(tf.global_variables_initializer())

        # Initialise target networks.
        actor_model.update_target_network()
        critic_model.update_target_network()

        # Replay is a simple list of lists, start of as an empty list.
        replay = []

        for epo in range(epochs):
            print('Game: %s' % epo)

            # Reset env and get initial observation.
            obs_1 = env.reset()

            # reward_total for output.
            reward_total = 0

            for j in range(run_length):
                # Render environment if possible.
                if render:
                    env.render()

                print('Obs: {}'.format(obs_1.reshape(1, state_dim)))

                # Calculate noise amount, set to half epsilon.
                noise_r = epsilon/2.
                # Calculate noisy action.
                action = actor_model.predict(obs_1.reshape(1, state_dim)) + random.uniform(-noise_r, noise_r)

                print('Act val: {} [{}, {}]'.format(action, epo, j))

                # Perform action.
                obs_2, reward, d, i = env.step(action)

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

                # Update epsilon, noise factor decreases each time.
                if epsilon > min_epsilon:
                    epsilon -= (1.0/(epochs*run_length))
                    # Ensure epsilon does not go below minimum.
                    if epsilon < min_epsilon:
                        epsilon = min_epsilon

                # For output info.
                reward_total += reward

                # In the invpen case this never happens, but left in for other
                # environments.
                if d:
                    continue

            print('Reward earned: {}'.format(reward_total))
            f.write('{:<10d}{:<f}\n'.format(epo, reward_total[0]))

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

        # Get initial epoch observations.
        obs_1 = env.reset()

        # Not strictly necessary...
        reward_total = 0

        for j in range(run_length):
            # Always draw in test mode.
            env.render()

            print('Obs: {}'.format(obs_1.reshape(1, state_dim)))

            # Get action, do not add noise.
            action = actor_model.predict(obs_1.reshape(1, state_dim))

            print('Act val: {} [{}, {}]'.format(action, epo, j))

            # Step and observe.
            obs_2, reward, d, i = env.step(action)

            # Move to our new state.
            obs_1 = obs_2[:]

            # Why is this even here?
            reward_total += reward

            if d:
                continue

        print('Reward earned: {}'.format(reward_total))

    # HEHEHHEHEHE
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

        # Get environment params, for building networks etc.
        state_dim = env.observation_space.shape[0]
        action_dim = env.action_space.shape[0]
        max_action = env.action_space.high

        print('{} {} {}'.format(state_dim, action_dim, max_action))

        # Build our actor and critic agents.
        actor_model = actor.ActorNetwork(sess, state_dim, action_dim, max_action, 0.0001, 0.001)
        critic_model = critic.CriticNetwork(sess, state_dim, action_dim, max_action, 0.001, 0.001, actor_model.get_num_trainable_vars())

        # Train.
        train(sess, actor_model, critic_model, env, state_dim, action_dim, max_action, epochs=50, run_length=300, render=False)

        raw_input('Training complete, press enter to continue to test.')

        # Test.
        test(sess, actor_model, critic_model, env, epochs=50, run_length=300)
