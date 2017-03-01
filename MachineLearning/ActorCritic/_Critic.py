import numpy as np

DISCOUNT_THRESHOLD = 0.001


class Critic:
    def __init__(self, state_dimensions, number_of_actions, time_horizon, discount, buffer_size):
        self.rewards = np.zeros(state_dimensions)
        self.timeHorizon = time_horizon
        self.discount = discount
        self.number_of_actions = number_of_actions

        self.bufferSize = buffer_size
        state_action_dims = state_dimensions + (number_of_actions,)
        action_knowledge_dims = state_action_dims + (len(state_dimensions),)
        self.state_action_observations = np.zeros(action_knowledge_dims, dtype=np.int_)
        observations_buffer_dims = state_action_dims + (len(state_dimensions) + 1, buffer_size)
        self.observation_buffer = np.zeros(observations_buffer_dims, dtype=np.int_)
        self.buffer_index = np.zeros(state_action_dims, dtype=np.int_)

    def update_action_knowledge(self, previous_state, previous_action, state):
        state_action = previous_state + (previous_action,)
        new_knowledge = np.asarray(state).T
        number_in_buffer = self.buffer_index[state_action]

        observation_location = -1
        # check for next state in buffer
        for i in range(0, number_in_buffer):
            index = state_action + (slice(1, 3), i)
            if np.array_equal(self.observation_buffer[index], new_knowledge):
                observation_location = i

        if observation_location < 0:
            if number_in_buffer < self.bufferSize:
                self.observation_buffer[state_action + (slice(1, 3), number_in_buffer)] = new_knowledge
                self.observation_buffer[state_action + (0, number_in_buffer)] += 1
                self.buffer_index[state_action] += 1
        else:
            self.observation_buffer[state_action + (0, observation_location)] += 1

        mode_index = np.argmax(self.observation_buffer[state_action + (0, Ellipsis)])
        mode_action = self.observation_buffer[state_action + (slice(1, 3), mode_index)]

        self.state_action_observations[state_action] = mode_action

    def get_td_error(self, previous_state, new_state, policy, action_knowledge):
        r1 = self.rewards[new_state]
        # print "Reward of new state: ", r1
        v1 = self.value(new_state, policy)
        # print "Value of new state: ", v1
        v0 = self.value(previous_state, policy)
        # print "Value of previous state: ", v0
        return r1 + self.discount * v1 - v0

    def value(self, state, policy):
        value = 0
        for t in range(1, self.timeHorizon+1):
            # Get next state
            action = np.argmax(policy[state])
            next_state_array = self.state_action_observations[state + (action,)]
            next_state = tuple([int(round(stateVar)) for stateVar in next_state_array])
            reward = self.rewards[next_state]
            value += self.discount**t * reward

            state = next_state

        return value

    # def value(self, state, policy, discount_p):
    #     """
    #     :return: the value of the state i.e E[ sum_{t=0}^\inf( gamma^t * R_{t+1} ) ]
    #     """
    #     probabilities = policy[state]
    #     total_value = 0
    #     # print t
    #     for action in range(self.number_of_actions):
    #         probability = probabilities[action]
    #         if discount_p > DISCOUNT_THRESHOLD and self.buffer_index[state + (action,)] > 0:
    #             next_state = tuple(self.state_action_observations[state + (action,)].tolist())
    #             next_reward = self.rewards[next_state]
    #             total_value += probability * (
    #                 next_reward
    #                 + self.discount
    #                 * self.value(next_state, policy=policy, discount_p=probability * self.discount * discount_p)
    #             )
    #     return total_value

    def update_reward(self, state, reward):
        self.rewards[state] = reward
