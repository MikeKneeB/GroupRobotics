from copy import deepcopy
from keras.models import Sequential
import numpy as np
import time
import statistics

# the Q network, important stuff
class DeepQNetwork(object):

    # constructor initialises the network with the given parameters
    def __init__(self, model, policy, number_actions, memory, gamma, batch_size, environment):
        self.model = model
        self.policy = policy
        self.number_actions = number_actions
        self.memory = memory
        self.gamma = gamma
        self.batch_size = batch_size
        self.environment = environment

    # convenience method to convert to the correct input for the
    # environments step method
    # could do with being slightly more generic
    def process_action(self, action):

        correction = action/((self.number_actions-1)/4)
        action = -2 + correction # -2 is the min action and +2 is the max
        return [action]

    # convenience method to prepare for keras methods
    def process_state(self, state):
        state = np.array(state)
        return state

    # use the network to compute the Q-values for a state
    def compute_q_values(self, state):
        state=self.process_state(state)
        q_values = self.model.predict_on_batch(state).flatten()

        return q_values

    # compile the model so it can be used
    def compile(self, optimizer, metrics=[]):
        config = self.model.get_config()
        self.target_model = Sequential.from_config(config)
        self.target_model.compile(optimizer,loss='mse')
        self.model.compile(optimizer,loss='mse')
        self.target_model.set_weights(self.model.get_weights())

    # load network weights from a save file
    def load_weights(self, filepath):
        self.model.load_weights(filepath)

    # save network weights to a file
    def save_weights(self, filepath, overwrite=False):
        self.model.save_weights(filepath, overwrite=overwrite)

    # reset known state information
    def reset(self):
        self.last_observation = None
        self.last_action = None

    # select and action using the policy
    def select_action(self, state):
        q_values=self.compute_q_values([[state]])   # convert state into right form for keras
        action=self.policy.select_action(q_values=q_values)

        self.last_observation=state
        self.last_action=action

        return action

    # update the network with some experiences in one gradient update
    def update_net(self, reward, state):
        # add the most recent experience
        self.memory.add(self.last_observation, self.last_action, reward, state)

        # get a batch of experiences from memory
        start_state_batch, action_batch, reward_batch, final_state_batch = self.memory.sample(self.batch_size)

        # state batches are re-shaped to function with keras methods
        start_state_batch = np.array([start_state_batch]).reshape((-1, 1, self.environment.observation_space.shape[0]))
        final_state_batch = np.array([final_state_batch]).reshape((-1, 1, self.environment.observation_space.shape[0]))

        # compute the q-values for the next and current state
        next_state_q_values = self.target_model.predict_on_batch(final_state_batch)
        current_state_q_values = self.model.predict_on_batch(start_state_batch)

        # using Q-value algorithm compute new q value for state action pair
        q_batch = np.max(next_state_q_values, axis=1).flatten() # best action in next state
        discounted_reward_batch = self.gamma * q_batch # discount the action
        q_update = reward_batch + discounted_reward_batch # Q-values updates

        # replace the q value for the action taken with the new value
        for idx, (current_state, action, Q) in enumerate(zip(current_state_q_values, action_batch, q_update)):
            current_state[action] = Q

        # single gradient train with states and altered q-values
        self.model.train_on_batch(start_state_batch, current_state_q_values)

    # train the network with the environment
    def train(self, episodes, steps, visualize=False):
        # for each episode
        print('training for %s episodes' % episodes)
        file = open('episode_data_target.txt', 'w')
        start = time.time()
        for episode in range(episodes):

            # reset the environment
            current_state = deepcopy(self.environment.reset())
            episode_reward = 0
            # visualize if desired
            if visualize:
                self.environment.render()

            # print the current episode
            print("Episode #: %s" % (episode,))

            # for every step in an episode
            for step in range(steps):
                # select and perform an action
                action=self.select_action(current_state)
                new_state, reward, done, info = self.environment.step(self.process_action(action))
                episode_reward+=reward
                if visualize:
                    self.environment.render()

                # update the network with the results
                self.update_net(reward, new_state)
                if done:
                    break
                # progress the state of the environment
                current_state = deepcopy(new_state)

            file.write('%s\t%s\n' % (episode, episode_reward))
            if episode%1==0:
                self.target_model.set_weights(self.model.get_weights())
        end = time.time()
        file.close()
        print('Time to complete training of %s episodes: %r seconds' % (episodes,(end-start)))

    # test the trained network on the environment
    def test(self, episodes, steps, visualize=True):
        # for each episode
        print('Testing for %s episodes' % episodes)
        rewards = []
        for episode in range(episodes):
            episode_reward = 0
            # reset
            current_state = deepcopy(self.environment.reset())
            if visualize:
                self.environment.render()

            # print the episode
            print("Episode #: %s" % (episode,))
            # perform each step
            for step in range(steps):
                # select and perform an action
                action = self.select_action(current_state)
                new_state, reward, done, info = self.environment.step(self.process_action(action))
                episode_reward+=reward
                if visualize:
                    self.environment.render()
                if done:
                    break

                # progress the state of the environment
                current_state = deepcopy(new_state)
            rewards.append(episode_reward)
            print('Episode reward : %r' % episode_reward)
        print('Average reward : %r +/- %r' % (statistics.mean(rewards) , statistics.stdev(rewards)))